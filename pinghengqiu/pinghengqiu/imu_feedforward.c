#include "imu_feedforward.h"

#include <math.h>
#include <stdint.h>

#include "MPU6050.h"
#include "clock.h"
#include "ti_msp_dl_config.h"

#define IMU_FF_SAMPLE_PERIOD_MS          5UL
#define IMU_FF_SAMPLE_PERIOD_S           0.005f

#define IMU_FF_CALIBRATION_SAMPLES       400U
#define IMU_FF_MAX_READ_ERRORS           20U

#define IMU_FF_ACCEL_SCALE_LSB_G         8192.0f
#define IMU_FF_GYRO_SCALE_LSB_DPS        65.5f

#define IMU_FF_GRAVITY_MPS2              9.80665f
#define IMU_FF_RAD_TO_DEG                57.2957795f
#define IMU_FF_DEG_TO_RAD                0.0174532925f

/*
 * 互补滤波系数。
 *
 * 越接近 1，越信任陀螺仪，越不容易把小车线性加速度
 * 错误识别成车体倾斜。
 */
#define IMU_FF_COMPLEMENTARY_ALPHA       0.995f

/*
 * 纵向加速度低通滤波截止频率约 8Hz。
 *
 * dt = 0.005s
 * RC = 1 / (2*pi*8) = 0.0199s
 * alpha = dt / (RC + dt) 约 0.20
 */
#define IMU_FF_ACCEL_LOWPASS_ALPHA       0.20f

/*
 * 坐标映射：
 *
 *  1  = MPU X
 * -1  = -MPU X
 *  2  = MPU Y
 * -2  = -MPU Y
 *  3  = MPU Z
 * -3  = -MPU Z
 *
 * 默认假设：
 * MPU X 朝小车前方；
 * MPU Y 朝小车侧方；
 * MPU Z 朝上。
 */
#define IMU_BODY_X_SOURCE                1
#define IMU_BODY_Y_SOURCE                2
#define IMU_BODY_Z_SOURCE                3

typedef struct {
    volatile ImuFeedforwardState_t state;

    unsigned long last_process_ms;

    float accel_bias_x_g;
    float accel_bias_y_g;
    float accel_bias_z_g;

    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;

    double calibration_accel_sum_x;
    double calibration_accel_sum_y;
    double calibration_accel_sum_z;

    double calibration_gyro_sum_x;
    double calibration_gyro_sum_y;
    double calibration_gyro_sum_z;

    uint32_t calibration_count;

    float pitch_deg;
    uint8_t pitch_initialized;

    float filtered_acceleration_mps2;

    volatile float feedforward_angle_deg;
    volatile float longitudinal_acceleration_mps2;

    float feedforward_gain;
    float angle_limit_deg;
    float acceleration_deadband_mps2;
    int8_t direction_sign;

    uint32_t sample_count;
    uint32_t read_error_count;

    ImuFeedforwardSnapshot_t snapshot;
} ImuFeedforwardContext_t;

static ImuFeedforwardContext_t s_imu;

static float ImuFeedforward_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static float ImuFeedforward_Limit(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static float ImuFeedforward_SelectAxis(
    const float values[3],
    int source)
{
    int index;
    float sign;

    if (source < 0) {
        index = -source - 1;
        sign = -1.0f;
    } else {
        index = source - 1;
        sign = 1.0f;
    }

    if (index < 0 || index > 2) {
        return 0.0f;
    }

    return sign * values[index];
}

static void ImuFeedforward_MapAxes(
    const float sensor_values[3],
    float body_values[3])
{
    body_values[0] = ImuFeedforward_SelectAxis(
        sensor_values,
        IMU_BODY_X_SOURCE);

    body_values[1] = ImuFeedforward_SelectAxis(
        sensor_values,
        IMU_BODY_Y_SOURCE);

    body_values[2] = ImuFeedforward_SelectAxis(
        sensor_values,
        IMU_BODY_Z_SOURCE);
}

static void ImuFeedforward_ClearCalibration(void)
{
    s_imu.calibration_accel_sum_x = 0.0;
    s_imu.calibration_accel_sum_y = 0.0;
    s_imu.calibration_accel_sum_z = 0.0;

    s_imu.calibration_gyro_sum_x = 0.0;
    s_imu.calibration_gyro_sum_y = 0.0;
    s_imu.calibration_gyro_sum_z = 0.0;

    s_imu.calibration_count = 0U;

    s_imu.accel_bias_x_g = 0.0f;
    s_imu.accel_bias_y_g = 0.0f;
    s_imu.accel_bias_z_g = 0.0f;

    s_imu.gyro_bias_x_dps = 0.0f;
    s_imu.gyro_bias_y_dps = 0.0f;
    s_imu.gyro_bias_z_dps = 0.0f;

    s_imu.pitch_deg = 0.0f;
    s_imu.pitch_initialized = 0U;

    s_imu.filtered_acceleration_mps2 = 0.0f;
    s_imu.feedforward_angle_deg = 0.0f;
    s_imu.longitudinal_acceleration_mps2 = 0.0f;
}

static void ImuFeedforward_CalibrationUpdate(
    const float accel_g[3],
    const float gyro_dps[3])
{
    float average_accel_x;
    float average_accel_y;
    float average_accel_z;

    s_imu.calibration_accel_sum_x += accel_g[0];
    s_imu.calibration_accel_sum_y += accel_g[1];
    s_imu.calibration_accel_sum_z += accel_g[2];

    s_imu.calibration_gyro_sum_x += gyro_dps[0];
    s_imu.calibration_gyro_sum_y += gyro_dps[1];
    s_imu.calibration_gyro_sum_z += gyro_dps[2];

    s_imu.calibration_count++;

    if (s_imu.calibration_count <
        IMU_FF_CALIBRATION_SAMPLES) {
        return;
    }

    average_accel_x = (float)(
        s_imu.calibration_accel_sum_x /
        (double)IMU_FF_CALIBRATION_SAMPLES);

    average_accel_y = (float)(
        s_imu.calibration_accel_sum_y /
        (double)IMU_FF_CALIBRATION_SAMPLES);

    average_accel_z = (float)(
        s_imu.calibration_accel_sum_z /
        (double)IMU_FF_CALIBRATION_SAMPLES);

    /*
     * 校准时要求车体水平且静止。
     *
     * 期望：
     * X = 0g
     * Y = 0g
     * Z = +1g
     */
    s_imu.accel_bias_x_g = average_accel_x;
    s_imu.accel_bias_y_g = average_accel_y;
    s_imu.accel_bias_z_g = average_accel_z - 1.0f;

    s_imu.gyro_bias_x_dps = (float)(
        s_imu.calibration_gyro_sum_x /
        (double)IMU_FF_CALIBRATION_SAMPLES);

    s_imu.gyro_bias_y_dps = (float)(
        s_imu.calibration_gyro_sum_y /
        (double)IMU_FF_CALIBRATION_SAMPLES);

    s_imu.gyro_bias_z_dps = (float)(
        s_imu.calibration_gyro_sum_z /
        (double)IMU_FF_CALIBRATION_SAMPLES);

    s_imu.pitch_deg = 0.0f;
    s_imu.pitch_initialized = 1U;

    s_imu.filtered_acceleration_mps2 = 0.0f;
    s_imu.feedforward_angle_deg = 0.0f;
    s_imu.longitudinal_acceleration_mps2 = 0.0f;

    s_imu.state = IMU_FF_STATE_READY;
}

static void ImuFeedforward_UpdateControl(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    float gyro_y_dps)
{
    float accel_pitch_deg;
    float predicted_pitch_deg;
    float pitch_rad;
    float gravity_x_g;

    float acceleration_mps2;
    float filtered_acceleration;
    float effective_acceleration;

    float feedforward_angle_deg;

    /*
     * 加速度计估计俯仰角。
     *
     * 小车快速加减速时该角度会受线性加速度影响，
     * 所以只能低权重参与互补滤波。
     */
    accel_pitch_deg =
        atan2f(
            -accel_x_g,
            sqrtf(
                accel_y_g * accel_y_g +
                accel_z_g * accel_z_g)) *
        IMU_FF_RAD_TO_DEG;

    if (s_imu.pitch_initialized == 0U) {
        s_imu.pitch_deg = accel_pitch_deg;
        s_imu.pitch_initialized = 1U;
    } else {
        predicted_pitch_deg =
            s_imu.pitch_deg +
            gyro_y_dps * IMU_FF_SAMPLE_PERIOD_S;

        s_imu.pitch_deg =
            IMU_FF_COMPLEMENTARY_ALPHA *
                predicted_pitch_deg +
            (1.0f - IMU_FF_COMPLEMENTARY_ALPHA) *
                accel_pitch_deg;
    }

    pitch_rad =
        s_imu.pitch_deg *
        IMU_FF_DEG_TO_RAD;

    /*
     * 当车体有俯仰角时，重力会投影到车体 X 轴。
     *
     * 默认坐标方向下：
     * gravity_x_g = -sin(pitch)
     *
     * MPU6050 测量的是比力，因此小车向前加速时，
     * X 轴通常表现为负方向变化。
     */
    gravity_x_g = -sinf(pitch_rad);

    acceleration_mps2 =
        (gravity_x_g - accel_x_g) *
        IMU_FF_GRAVITY_MPS2;

    /*
     * 一阶低通滤波。
     */
    filtered_acceleration =
        s_imu.filtered_acceleration_mps2 +
        IMU_FF_ACCEL_LOWPASS_ALPHA *
        (acceleration_mps2 -
         s_imu.filtered_acceleration_mps2);

    s_imu.filtered_acceleration_mps2 =
        filtered_acceleration;

    /*
     * 死区处理。
     */
    if (ImuFeedforward_Abs(filtered_acceleration) <
        s_imu.acceleration_deadband_mps2) {
        effective_acceleration = 0.0f;
    } else if (filtered_acceleration > 0.0f) {
        effective_acceleration =
            filtered_acceleration -
            s_imu.acceleration_deadband_mps2;
    } else {
        effective_acceleration =
            filtered_acceleration +
            s_imu.acceleration_deadband_mps2;
    }

    /*
     * 理论补偿角：
     *
     * theta = atan(a / g)
     *
     * direction_sign 用于适配摆杆、电机和 MPU 安装方向。
     */
    feedforward_angle_deg =
        (float)s_imu.direction_sign *
        s_imu.feedforward_gain *
        atanf(
            effective_acceleration /
            IMU_FF_GRAVITY_MPS2) *
        IMU_FF_RAD_TO_DEG;

    feedforward_angle_deg =
        ImuFeedforward_Limit(
            feedforward_angle_deg,
            -s_imu.angle_limit_deg,
            s_imu.angle_limit_deg);

    s_imu.longitudinal_acceleration_mps2 =
        filtered_acceleration;

    s_imu.feedforward_angle_deg =
        feedforward_angle_deg;
}

uint8_t ImuFeedforward_Init(void)
{
    s_imu.last_process_ms = tick_ms;

    s_imu.feedforward_gain = 0.20f;
    s_imu.angle_limit_deg = 3.0f;
    s_imu.acceleration_deadband_mps2 = 0.10f;
    s_imu.direction_sign = 1;

    s_imu.sample_count = 0U;
    s_imu.read_error_count = 0U;

    ImuFeedforward_ClearCalibration();

    if (!MPU6050_Init()) {
        s_imu.state = IMU_FF_STATE_FAULT;
        return 0U;
    }

    s_imu.state = IMU_FF_STATE_CALIBRATING;
    return 1U;
}

void ImuFeedforward_Process(void)
{
    unsigned long now;

    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;

    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    float sensor_accel[3];
    float sensor_gyro[3];

    float body_accel[3];
    float body_gyro[3];

    now = tick_ms;

    if ((unsigned long)(
            now - s_imu.last_process_ms) <
        IMU_FF_SAMPLE_PERIOD_MS) {
        return;
    }

    s_imu.last_process_ms = now;

    if (s_imu.state ==
        IMU_FF_STATE_NOT_INITIALIZED ||
        s_imu.state ==
        IMU_FF_STATE_FAULT) {
        return;
    }

    if (!MPU6050_GetData(
            &raw_accel_x,
            &raw_accel_y,
            &raw_accel_z,
            &raw_gyro_x,
            &raw_gyro_y,
            &raw_gyro_z)) {
        s_imu.read_error_count++;

        if (s_imu.read_error_count >=
            IMU_FF_MAX_READ_ERRORS) {
            s_imu.feedforward_angle_deg = 0.0f;
            s_imu.state = IMU_FF_STATE_FAULT;
        }

        return;
    }

    s_imu.read_error_count = 0U;
    s_imu.sample_count++;

    /*
     * 原始值换算：
     *
     * 加速度计 +/-4g：8192 LSB/g
     * 陀螺仪 +/-500dps：65.5 LSB/(deg/s)
     */
    sensor_accel[0] =
        (float)raw_accel_x /
        IMU_FF_ACCEL_SCALE_LSB_G;

    sensor_accel[1] =
        (float)raw_accel_y /
        IMU_FF_ACCEL_SCALE_LSB_G;

    sensor_accel[2] =
        (float)raw_accel_z /
        IMU_FF_ACCEL_SCALE_LSB_G;

    sensor_gyro[0] =
        (float)raw_gyro_x /
        IMU_FF_GYRO_SCALE_LSB_DPS;

    sensor_gyro[1] =
        (float)raw_gyro_y /
        IMU_FF_GYRO_SCALE_LSB_DPS;

    sensor_gyro[2] =
        (float)raw_gyro_z /
        IMU_FF_GYRO_SCALE_LSB_DPS;

    /*
     * 将 MPU 坐标转换成小车坐标。
     */
    ImuFeedforward_MapAxes(
        sensor_accel,
        body_accel);

    ImuFeedforward_MapAxes(
        sensor_gyro,
        body_gyro);

    if (s_imu.state ==
        IMU_FF_STATE_CALIBRATING) {
        ImuFeedforward_CalibrationUpdate(
            body_accel,
            body_gyro);

        return;
    }

    /*
     * 减去静态零偏。
     */
    body_accel[0] -= s_imu.accel_bias_x_g;
    body_accel[1] -= s_imu.accel_bias_y_g;
    body_accel[2] -= s_imu.accel_bias_z_g;

    body_gyro[0] -= s_imu.gyro_bias_x_dps;
    body_gyro[1] -= s_imu.gyro_bias_y_dps;
    body_gyro[2] -= s_imu.gyro_bias_z_dps;

    ImuFeedforward_UpdateControl(
        body_accel[0],
        body_accel[1],
        body_accel[2],
        body_gyro[1]);

    s_imu.snapshot.acceleration_x_g =
        body_accel[0];

    s_imu.snapshot.acceleration_y_g =
        body_accel[1];

    s_imu.snapshot.acceleration_z_g =
        body_accel[2];

    s_imu.snapshot.gyro_x_dps =
        body_gyro[0];

    s_imu.snapshot.gyro_y_dps =
        body_gyro[1];

    s_imu.snapshot.gyro_z_dps =
        body_gyro[2];

    s_imu.snapshot.pitch_deg =
        s_imu.pitch_deg;

    s_imu.snapshot.longitudinal_acceleration_mps2 =
        s_imu.longitudinal_acceleration_mps2;

    s_imu.snapshot.filtered_acceleration_mps2 =
        s_imu.filtered_acceleration_mps2;

    s_imu.snapshot.feedforward_angle_deg =
        s_imu.feedforward_angle_deg;

    s_imu.snapshot.sample_count =
        s_imu.sample_count;

    s_imu.snapshot.read_error_count =
        s_imu.read_error_count;

    s_imu.snapshot.state =
        s_imu.state;
}

void ImuFeedforward_StartCalibration(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    ImuFeedforward_ClearCalibration();
    s_imu.read_error_count = 0U;
    s_imu.state = IMU_FF_STATE_CALIBRATING;

    if (primask == 0U) {
        __enable_irq();
    }
}

void ImuFeedforward_SetGain(float gain)
{
    if (gain != gain) {
        return;
    }

    s_imu.feedforward_gain =
        ImuFeedforward_Limit(
            gain,
            0.0f,
            2.0f);
}

void ImuFeedforward_SetAngleLimit(
    float angle_limit_deg)
{
    if (angle_limit_deg != angle_limit_deg) {
        return;
    }

    s_imu.angle_limit_deg =
        ImuFeedforward_Limit(
            ImuFeedforward_Abs(angle_limit_deg),
            0.0f,
            8.0f);
}

void ImuFeedforward_SetDeadband(
    float deadband_mps2)
{
    if (deadband_mps2 != deadband_mps2) {
        return;
    }

    s_imu.acceleration_deadband_mps2 =
        ImuFeedforward_Limit(
            ImuFeedforward_Abs(deadband_mps2),
            0.0f,
            2.0f);
}

void ImuFeedforward_SetDirection(int8_t sign)
{
    s_imu.direction_sign =
        sign < 0 ? -1 : 1;
}

float ImuFeedforward_GetAngleDeg(void)
{
    if (s_imu.state != IMU_FF_STATE_READY) {
        return 0.0f;
    }

    return s_imu.feedforward_angle_deg;
}

float ImuFeedforward_GetAccelerationMps2(void)
{
    if (s_imu.state != IMU_FF_STATE_READY) {
        return 0.0f;
    }

    return s_imu.longitudinal_acceleration_mps2;
}

uint8_t ImuFeedforward_IsReady(void)
{
    return s_imu.state ==
        IMU_FF_STATE_READY;
}

ImuFeedforwardState_t ImuFeedforward_GetState(void)
{
    return s_imu.state;
}

void ImuFeedforward_GetSnapshot(
    ImuFeedforwardSnapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == 0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *snapshot = s_imu.snapshot;

    snapshot->feedforward_angle_deg =
        s_imu.feedforward_angle_deg;

    snapshot->longitudinal_acceleration_mps2 =
        s_imu.longitudinal_acceleration_mps2;

    snapshot->state = s_imu.state;

    if (primask == 0U) {
        __enable_irq();
    }
}