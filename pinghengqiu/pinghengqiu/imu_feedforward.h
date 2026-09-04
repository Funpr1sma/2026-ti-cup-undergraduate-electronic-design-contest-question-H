#ifndef IMU_FEEDFORWARD_H
#define IMU_FEEDFORWARD_H

#include <stdint.h>

typedef enum {
    IMU_FF_STATE_NOT_INITIALIZED = 0,
    IMU_FF_STATE_CALIBRATING,
    IMU_FF_STATE_READY,
    IMU_FF_STATE_FAULT
} ImuFeedforwardState_t;

typedef struct {
    float acceleration_x_g;
    float acceleration_y_g;
    float acceleration_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float pitch_deg;

    float longitudinal_acceleration_mps2;
    float filtered_acceleration_mps2;

    float feedforward_angle_deg;

    uint32_t sample_count;
    uint32_t read_error_count;

    ImuFeedforwardState_t state;
} ImuFeedforwardSnapshot_t;

/*
 * 在 SYSCFG_DL_init() 和 SysTick_Init() 之后调用。
 */
uint8_t ImuFeedforward_Init(void);

/*
 * 在主循环中持续调用。
 * 函数内部限制为每 5ms 读取一次 MPU6050。
 *
 * 不要在定时器中断中调用，因为 I2C 读取是阻塞式的。
 */
void ImuFeedforward_Process(void);

/*
 * 重新开始静态校准。
 * 调用后必须保持小车静止、水平约 2 秒。
 */
void ImuFeedforward_StartCalibration(void);

/*
 * 设置前馈增益。
 * 建议从 0.2 开始，逐渐增加。
 */
void ImuFeedforward_SetGain(float gain);

/*
 * 设置前馈角度限幅。
 * 建议先使用 2~3 度。
 */
void ImuFeedforward_SetAngleLimit(float angle_limit_deg);

/*
 * 设置加速度死区，单位 m/s^2。
 */
void ImuFeedforward_SetDeadband(float deadband_mps2);

/*
 * 设置前馈方向。
 * sign 只能使用 1 或 -1。
 */
void ImuFeedforward_SetDirection(int8_t sign);

float ImuFeedforward_GetAngleDeg(void);
float ImuFeedforward_GetAccelerationMps2(void);

uint8_t ImuFeedforward_IsReady(void);
ImuFeedforwardState_t ImuFeedforward_GetState(void);

void ImuFeedforward_GetSnapshot(
    ImuFeedforwardSnapshot_t *snapshot);

#endif
