#include "VofaTuning.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "CameraUart.h"
#include "BalanceLevelConfig.h"
#include "CarMotionRx.h"
#include "Req4VisualControl.h"
#include "Key.h"
#include "Serial.h"
#include "ball_balance_control.h"
#include "ball_link.h"
#include "closed_loop.h"
#include "encoder.h"
#include "imu_feedforward.h"
#include "motor.h"

#define VOFA_COMMAND_BUFFER_SIZE   128U
#define VOFA_RX_BYTES_PER_CALL      64U
#define VOFA_PLOT_DEFAULT_MS        50U
#define VOFA_PLOT_MIN_MS            20U
#define VOFA_PLOT_MAX_MS          1000U

typedef enum {
    VOFA_PLOT_OFF = 0,
    VOFA_PLOT_BALANCE = 1,
    VOFA_PLOT_IMU = 2,
    VOFA_PLOT_MOTOR = 3,
    VOFA_PLOT_REQ4 = 4
} VofaPlotMode_t;

typedef enum {
    TEST3_IDLE = 0,
    TEST3_TO_POSITIVE,
    TEST3_TO_NEGATIVE,
    TEST3_FINISHED
} Test3State_t;

typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_TO_NEGATIVE,
    MOTOR_TEST_WAIT_NEGATIVE,
    MOTOR_TEST_TO_POSITIVE
} MotorTestState_t;

typedef enum {
    LEVEL_MOVE_IDLE = 0,
    LEVEL_MOVE_TO_DEFAULT,
    LEVEL_MOVE_SETTLING
} LevelMoveState_t;

#define MOTOR_DEBUG_LIMIT_DEG       900.0f
#define MOTOR_TEST_NEGATIVE_DEG    (-900.0f)
#define MOTOR_TEST_POSITIVE_DEG     900.0f
#define MOTOR_TEST_SETTLE_MS        300U

/*
 * Saved level position is expressed in encoder raw angle from this power-up.
 * The default comes from BalanceLevelConfig.h. With an incremental encoder,
 * a value saved in RAM cannot identify an absolute mechanical position after
 * power loss; the default therefore assumes the mechanism powers up near its
 * known level position. "levelsave" remains available for session calibration.
 */
#define VOFA_LEVEL_MOVE_SETTLE_MS        100U
#define VOFA_LEVEL_MOVE_LIMIT_DEG       1700.0f

/* PRETILT_READY is returned only after the actual screw position is close. */
#define VOFA_PRETILT_MOTOR_WINDOW_DEG     70.0f
#define VOFA_PRETILT_MIN_FF_ANGLE_DEG      0.08f

/*
 * H题要求3：全部位置均按摄像头输出的钢球球心坐标判断。
 * 不加入钢球半径，也不以球身边缘接触刻度作为到达条件。
 */
#define TEST3_POSITIVE_TARGET_CM       5.0f
#define TEST3_NEGATIVE_TARGET_CM      (-5.0f)
#define TEST3_POSITION_LIMIT_CM         0.50f
#define TEST3_POSITIVE_TURN_MIN_CM      4.50f
#define TEST3_POSITIVE_TURN_MAX_CM      5.50f
#define TEST3_POSITIVE_CONFIRM_MS        0U
#define TEST3_POSITIVE_STAGE_LIMIT_MS  3000U
#define TEST3_TOTAL_LIMIT_MS           5000U

static char s_command_buffer[VOFA_COMMAND_BUFFER_SIZE];
static uint16_t s_command_length;
static uint8_t s_command_overflow;
static VofaPlotMode_t s_plot_mode;
static uint32_t s_plot_period_ms;
static uint32_t s_last_plot_ms;
static Test3State_t s_test_state;
static uint32_t s_test_start_ms;
static uint32_t s_test_stage_ms;
static uint32_t s_test_positive_confirm_ms;
static uint8_t s_test_positive_confirm_active;
static uint8_t s_test_positive_reached;
static float s_test_positive_turn_cm;
static float s_test_positive_error_cm;
static MotorTestState_t s_motor_test_state;
static uint32_t s_motor_test_start_ms;
static uint32_t s_motor_test_wait_ms;
static LevelMoveState_t s_level_move_state;
static uint32_t s_level_move_settle_ms;
static float s_level_default_raw_deg;
static uint8_t s_level_default_valid;

/* Car-button-driven requirement 4/5/6 integration state. */
static uint8_t s_remote_pending_mission;
static uint8_t s_remote_active_mission;
static uint8_t s_remote_target_latched;
static float s_remote_captured_target_cm;
static uint32_t s_remote_last_retry_ms;

static void Vofa_CancelMotorTest(void);
static void Vofa_ProcessMotorTest(uint32_t now_ms);
static void Vofa_CancelLevelMove(void);
static void Vofa_ProcessLevelMove(uint32_t now_ms);
static void Vofa_StartRequirement4(uint32_t now_ms);
static void Vofa_StartRequirement4VisualOnly(uint32_t now_ms);
static void Vofa_StartRequirement6(uint32_t now_ms);
static void Vofa_ProcessCarMission(uint32_t now_ms);
static void Vofa_UpdateCarAck(uint32_t now_ms);
static void Vofa_ClearRemoteMission(void);
static void Vofa_PrintCarLink(void);
static void Vofa_PrintMotorState(void);
static void Vofa_PrintReq4Config(void);
static void Vofa_PrintReq4Status(void);

static char Vofa_ToLower(char value)
{
    if (value >= 'A' && value <= 'Z') {
        return (char)(value - 'A' + 'a');
    }
    return value;
}

static char *Vofa_Normalize(char *text)
{
    char *start;
    char *end;
    char *cursor;
    uint8_t replaced_separator = 0U;

    if (text == NULL) {
        return NULL;
    }

    start = text;
    while (*start == ' ' || *start == '\t') {
        start++;
    }

    if (*start == '[') {
        start++;
    }

    end = start + strlen(start);
    while (end > start &&
           (end[-1] == ' ' || end[-1] == '\t' || end[-1] == ']')) {
        end--;
    }
    *end = '\0';

    for (cursor = start; *cursor != '\0'; cursor++) {
        *cursor = Vofa_ToLower(*cursor);
        /* Compatibility with old Bluetooth commands such as [target,5]. */
        if (*cursor == ',' && replaced_separator == 0U) {
            *cursor = '=';
            replaced_separator = 1U;
        }
    }

    return start;
}

static uint8_t Vofa_ParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    if (text == NULL || value == NULL || *text == '\0') {
        return 0U;
    }

    parsed = strtof(text, &end);
    while (*end == ' ' || *end == '\t') {
        end++;
    }

    if (end == text || *end != '\0' || parsed != parsed ||
        parsed < -1000000.0f || parsed > 1000000.0f) {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

static uint8_t Vofa_ParseU32(const char *text, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if (text == NULL || value == NULL || *text == '\0' || *text == '-') {
        return 0U;
    }

    parsed = strtoul(text, &end, 10);
    while (*end == ' ' || *end == '\t') {
        end++;
    }

    if (end == text || *end != '\0') {
        return 0U;
    }

    *value = (uint32_t)parsed;
    return 1U;
}

static uint8_t Vofa_ParseAssignment(
    char *command,
    const char *name,
    char **value_text)
{
    size_t name_length;

    if (command == NULL || name == NULL || value_text == NULL) {
        return 0U;
    }

    name_length = strlen(name);
    if (strncmp(command, name, name_length) != 0 ||
        command[name_length] != '=') {
        return 0U;
    }

    *value_text = command + name_length + 1U;
    return 1U;
}

static const char *Vofa_BalanceStateName(BallBalanceState_t state)
{
    switch (state) {
        case BALL_BALANCE_STATE_NOT_READY: return "not_ready";
        case BALL_BALANCE_STATE_DISABLED: return "disabled";
        case BALL_BALANCE_STATE_RETURNING_ZERO: return "returning_zero";
        case BALL_BALANCE_STATE_RUNNING: return "running";
        case BALL_BALANCE_STATE_SETTLED: return "settled";
        case BALL_BALANCE_STATE_FAULT: return "fault";
        default: return "unknown";
    }
}

static const char *Vofa_FaultName(BallBalanceFault_t fault)
{
    switch (fault) {
        case BALL_BALANCE_FAULT_NONE: return "none";
        case BALL_BALANCE_FAULT_CAMERA_TIMEOUT: return "camera_timeout";
        case BALL_BALANCE_FAULT_BALL_OUT_OF_RANGE: return "ball_out_of_range";
        case BALL_BALANCE_FAULT_ACTUATOR_OVER_TRAVEL: return "actuator_over_travel";
        case BALL_BALANCE_FAULT_MOTOR_ENCODER: return "motor_encoder";
        case BALL_BALANCE_FAULT_MOTOR_DIRECTION: return "motor_direction";
        case BALL_BALANCE_FAULT_MOTOR_DRIVER: return "motor_driver";
        case BALL_BALANCE_FAULT_INVALID_CONFIG: return "invalid_config";
        default: return "unknown";
    }
}

static const char *Vofa_ImuStateName(ImuFeedforwardState_t state)
{
    switch (state) {
        case IMU_FF_STATE_NOT_INITIALIZED: return "not_initialized";
        case IMU_FF_STATE_CALIBRATING: return "calibrating";
        case IMU_FF_STATE_READY: return "ready";
        case IMU_FF_STATE_FAULT: return "fault";
        default: return "unknown";
    }
}

static void Vofa_PrintHelp(void)
{
    Serial_SendString(
        "\r\n========== BALANCE BOARD VOFA+ ==========\r\n"
        "Use UART0 PB0(TX)/PB1(RX), 115200 8N1.\r\n"
        "Every command ends with CR/LF or LF.\r\n"
        "help | ping | serial | status | config | camera | camping | motor | imu | carlink\r\n"
        "mangle/angle       print relative/raw motor angle and saved level\r\n"
        "motorstate/mstate  detailed motor, encoder, target and level-move state\r\n"
        "level              current actuator position = level zero\r\n"
        "levelsave          save current raw angle and set it as level zero\r\n"
        "leveldefault=120   set saved raw level angle for this power-up\r\n"
        "levelgo            move to configured/session level angle, then auto-zero\r\n"
        "PB4                same as levelgo: return to default and define horizontal\r\n"
        "levelto=120         set saved angle and execute levelgo\r\n"
        "enable=1 / enable=0  enable or disable ball controller\r\n"
        "target=0             target ball position, cm\r\n"
        "sample=0             inject a manual ball position for bench test\r\n"
        "kp/ki/kd: original requirement-3/general controller parameters\r\n"
        "kp=0.18 | ki=0 | kd=0.10\r\n"
        "sign=1/-1 | arm=235 | maxangle=1.50 | travel=7\r\n"
        "timeout=1000 | clear | stop | test3 | req45 | req6\r\n"
        "test3: automatic O -> +5 -> -5; car feed-forward forced OFF\r\n"
        "req45/req4/centerff: target 0 cm, visual + car acceleration feed-forward\r\n"
        "req4visual: target 0 cm with feed-forward OFF; req6 captures current ball coordinate\r\n"
        "req4status | req4config | r4reset | r4defaults\r\n"
        "r4kp=0.20 r4ki=0 r4kd=0.06 r4predict=0.08 r4predlimit=0.8\r\n"
        "r4vfilter=0.22 r4max=0.90 r4ilim=0.10 r4slew=8.0\r\n"
        "r4dead=0.08 r4holdband=0.75 r4breakerr=0.35 r4breakspeed=0.25\r\n"
        "r4breakangle=0.55; inside holdband + low speed, breakaway is inhibited\r\n"
        "PB8(active-low to GND): req3 start/stop; PB5(active-low): save current 0deg level\r\n"
        "mzero | mstop | mclear | mdir\r\n"
        "mtarget=90           manual motor target, limited to +/-900 deg\r\n"
        "mtest                motor travel test: -900 deg to +900 deg\r\n"
        "plot=0               waveform off\r\n"
        "plot=1               balance CSV waveform\r\n"
        "plot=2               MPU6050 CSV waveform\r\n"
        "plot=3               motor target/current/error CSV waveform\r\n"
        "plot=4               req4: target,ball,error,speed,visual,accel,ff,total...\r\n"
        "period=50            waveform period 20..1000 ms\r\n"
        "imucal | imugain=0.2 | imulimit=3 | imudead=0.1 | imudir=1/-1\r\n"
        "carff=0/1 | carffgain=0.85 | carfflimit=1.20 | carffdead=0.015\r\n"
        "carffdir=1/-1 | carffconfig | carreset\r\n"
        "camping              send $PING* to MaixCAM2 on UART2\r\n"
        "camreset             reset camera UART/parser diagnostics\r\n"
        "Old forms such as [target,5] are also accepted.\r\n"
        "==========================================\r\n");
}

static void Vofa_PrintSerial(void)
{
    Serial_Printf(
        "SERIAL UART0 TX=PB0 RX=PB1 baud=115200 rx_irq=%lu "
        "rx_overflow=%lu tx_drop=%lu\r\n",
        (unsigned long)Serial_GetRxIrqCount(),
        (unsigned long)Serial_GetRxOverflowCount(),
        (unsigned long)Serial_GetTxDropCount());
}

static void Vofa_PrintStatus(void)
{
    BallBalanceSnapshot_t snapshot;

    BallBalance_GetSnapshot(&snapshot);
    Serial_Printf(
        "STATUS state=%s fault=%s enable=%u level=%u settled=%u r4=%u "
        "target=%.3f ball=%.3f error=%.3f speed=%.3f "
        "visual=%.3f car_a=%.3f ff=%.3f flap=%.3f "
        "ff_en=%u ff_ok=%u in1cm=%u lift=%.3f motor_target=%.3f motor=%.3f\r\n",
        Vofa_BalanceStateName(snapshot.state),
        Vofa_FaultName(snapshot.fault),
        (unsigned)snapshot.enabled,
        (unsigned)snapshot.level_calibrated,
        (unsigned)snapshot.settled,
        (unsigned)snapshot.requirement4_mode,
        (double)snapshot.target_ball_cm,
        (double)snapshot.current_ball_cm,
        (double)snapshot.ball_error_cm,
        (double)snapshot.ball_velocity_cm_s,
        (double)snapshot.visual_flap_target_deg,
        (double)snapshot.feedforward_acceleration_mps2,
        (double)snapshot.feedforward_angle_deg,
        (double)snapshot.flap_target_deg,
        (unsigned)snapshot.feedforward_enabled,
        (unsigned)snapshot.feedforward_valid,
        (unsigned)snapshot.req4_in_one_cm_band,
        (double)snapshot.actuator_target_mm,
        (double)snapshot.motor_target_deg,
        (double)snapshot.motor_current_deg);
}

static void Vofa_PrintConfig(void)
{
    BallBalanceConfig_t config;

    BallBalance_GetConfig(&config);
    Serial_Printf(
        "CONFIG kp=%.4f ki=%.4f kd=%.4f sign=%d arm=%.2f "
        "degmm=%.2f maxangle=%.2f travel=%.2f..%.2f timeout=%lu\r\n",
        (double)config.kp,
        (double)config.ki,
        (double)config.kd,
        (int)config.control_sign,
        (double)config.lever_arm_mm,
        (double)config.motor_deg_per_mm,
        (double)config.max_flap_angle_deg,
        (double)config.actuator_min_mm,
        (double)config.actuator_max_mm,
        (unsigned long)config.camera_timeout_ms);
}

static void Vofa_PrintReq4Config(void)
{
    Req4VisualConfig_t config;

    Req4VisualControl_GetConfig(&config);
    Serial_Printf(
        "REQ4_CONFIG kp=%.4f ki=%.4f kd=%.4f predict=%.3f predlimit=%.3f "
        "vfilter=%.3f visualmax=%.3f ilim=%.3f slew=%.3f "
        "dead=%.3f holdband=%.3f breakerr=%.3f breakspeed=%.3f breakangle=%.3f\r\n",
        (double)config.kp_deg_per_cm,
        (double)config.ki_deg_per_cm_s,
        (double)config.kd_deg_per_cm_s,
        (double)config.predict_time_s,
        (double)config.predict_limit_cm,
        (double)config.velocity_filter_alpha,
        (double)config.max_visual_angle_deg,
        (double)config.integral_limit_deg,
        (double)config.output_slew_deg_per_s,
        (double)config.center_deadband_cm,
        (double)config.breakaway_inhibit_band_cm,
        (double)config.breakaway_error_cm,
        (double)config.breakaway_speed_cm_s,
        (double)config.breakaway_angle_deg);
}

static void Vofa_PrintReq4Status(void)
{
    BallBalanceSnapshot_t balance;
    Req4VisualSnapshot_t visual;
    CarMotionRxSnapshot_t link;

    BallBalance_GetSnapshot(&balance);
    Req4VisualControl_GetSnapshot(&visual);
    CarMotionRx_GetSnapshot(&link);

    /*
     * Keep each formatted line below Serial_Printf's 256-byte local buffer.
     * The previous single long status line could be silently truncated.
     */
    Serial_Printf(
        "REQ4_STATUS1 mode=%u enable=%u state=%s fault=%s cam_seq=%lu "
        "target=%.3f ball=%.3f err=%.3f raw_v=%.3f filt_v=%.3f "
        "pred_err=%.3f p=%.3f i=%.3f d=%.3f break=%.3f visual=%.3f\r\n",
        (unsigned)balance.requirement4_mode,
        (unsigned)balance.enabled,
        Vofa_BalanceStateName(balance.state),
        Vofa_FaultName(balance.fault),
        (unsigned long)balance.measurement_sequence,
        (double)balance.target_ball_cm,
        (double)balance.current_ball_cm,
        (double)balance.ball_error_cm,
        (double)visual.raw_velocity_cm_s,
        (double)visual.filtered_velocity_cm_s,
        (double)visual.predicted_error_cm,
        (double)visual.p_output_deg,
        (double)visual.i_output_deg,
        (double)visual.d_output_deg,
        (double)visual.breakaway_output_deg,
        (double)balance.visual_flap_target_deg);

    Serial_Printf(
        "REQ4_STATUS2 carlink=%u motion=%u running=%u "
        "car_cps=%d car_accel=%d ff=%.3f total=%.3f "
        "motor_target=%.3f motor=%.3f in1cm=%u break_active=%u "
        "break_inhibit=%u sat=%u\r\n",
        (unsigned)link.link_valid,
        (unsigned)link.motion_active,
        (unsigned)link.running_active,
        (int)link.planned_cps,
        (int)link.acceleration_mm_s2,
        (double)balance.feedforward_angle_deg,
        (double)balance.flap_target_deg,
        (double)balance.motor_target_deg,
        (double)balance.motor_current_deg,
        (unsigned)balance.req4_in_one_cm_band,
        (unsigned)visual.breakaway_active,
        (unsigned)visual.breakaway_inhibited,
        (unsigned)balance.req4_visual_saturated);
}

static uint8_t Vofa_UpdateReq4Float(const char *name, const char *value_text)
{
    Req4VisualConfig_t config;
    float value;

    if (Vofa_ParseFloat(value_text, &value) == 0U) {
        return 0U;
    }

    Req4VisualControl_GetConfig(&config);
    if (strcmp(name, "r4kp") == 0) {
        config.kp_deg_per_cm = value;
    } else if (strcmp(name, "r4ki") == 0) {
        config.ki_deg_per_cm_s = value;
    } else if (strcmp(name, "r4kd") == 0) {
        config.kd_deg_per_cm_s = value;
    } else if (strcmp(name, "r4predict") == 0) {
        config.predict_time_s = value;
    } else if (strcmp(name, "r4predlimit") == 0) {
        config.predict_limit_cm = value;
    } else if (strcmp(name, "r4vfilter") == 0) {
        config.velocity_filter_alpha = value;
    } else if (strcmp(name, "r4max") == 0) {
        config.max_visual_angle_deg = value;
    } else if (strcmp(name, "r4ilim") == 0) {
        config.integral_limit_deg = value;
    } else if (strcmp(name, "r4slew") == 0) {
        config.output_slew_deg_per_s = value;
    } else if (strcmp(name, "r4dead") == 0) {
        config.center_deadband_cm = value;
    } else if (strcmp(name, "r4holdband") == 0) {
        config.breakaway_inhibit_band_cm = value;
    } else if (strcmp(name, "r4breakerr") == 0) {
        config.breakaway_error_cm = value;
    } else if (strcmp(name, "r4breakspeed") == 0) {
        config.breakaway_speed_cm_s = value;
    } else if (strcmp(name, "r4breakangle") == 0) {
        config.breakaway_angle_deg = value;
    } else {
        return 0U;
    }

    return Req4VisualControl_SetConfig(&config);
}

static void Vofa_PrintCamera(void)
{
    BallLinkContext context;
    CameraUartDiagnostics_t uart;
    uint8_t b[CAMERA_UART_RECENT_BYTE_COUNT] = {0U};
    uint32_t i;

    BallLink_GetContext(&context);
    CameraUart_GetDiagnostics(&uart);
    for (i = 0U; i < CAMERA_UART_RECENT_BYTE_COUNT; i++) {
        b[i] = uart.recent[i];
    }

    Serial_Printf(
        "CAM_UART PB15_TX/PB16_RX 115200 raw=%lu irq=%lu poll=%lu "
        "tx=%lu txdrop=%lu pong=%lu last=0x%02X maxdrain=%lu "
        "recent=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        (unsigned long)uart.rx_byte_count,
        (unsigned long)uart.irq_count,
        (unsigned long)uart.poll_drain_count,
        (unsigned long)uart.tx_byte_count,
        (unsigned long)uart.tx_drop_count,
        (unsigned long)uart.pong_count,
        (unsigned)uart.last_byte,
        (unsigned long)uart.max_drain_bytes,
        (unsigned)b[0], (unsigned)b[1], (unsigned)b[2], (unsigned)b[3],
        (unsigned)b[4], (unsigned)b[5], (unsigned)b[6], (unsigned)b[7]);

    Serial_Printf(
        "CAM_FRAME valid=%lu legacy=%lu extended=%lu invalid=%lu "
        "checksum=%lu duplicate=%lu parser_index=%u seq=%u flags=0x%02X "
        "ball_n=%u target_n=%u "
        "score=%u mode=%u ball_cm=%.3f\r\n",
        (unsigned long)context.valid_frame_count,
        (unsigned long)context.legacy_frame_count,
        (unsigned long)context.extended_frame_count,
        (unsigned long)context.invalid_frame_count,
        (unsigned long)context.checksum_error_count,
        (unsigned long)context.duplicate_frame_count,
        (unsigned)context.index,
        (unsigned)context.last_data.sequence,
        (unsigned)context.last_data.flags,
        (unsigned)context.last_data.ball_position,
        (unsigned)context.last_data.target_position,
        (unsigned)context.last_data.score_permille,
        (unsigned)context.last_data.mode,
        (double)context.last_position_cm);
}

static void Vofa_PrintMotor(void)
{
    CL_Snapshot_t snapshot;

    CL_GetSnapshot(MOTOR_AXIS_X, &snapshot);
    Serial_Printf(
        "MOTOR active=%u reached=%u fault=%u count=%ld target_count=%ld "
        "error=%ld angle=%.3f target_angle=%.3f\r\n",
        (unsigned)snapshot.active,
        (unsigned)snapshot.reached,
        (unsigned)snapshot.fault,
        (long)snapshot.current_count,
        (long)snapshot.target_count,
        (long)snapshot.error_count,
        (double)snapshot.current_angle_deg,
        (double)snapshot.target_angle_deg);
}

static void Vofa_PrintMotorAngle(void)
{
    CL_Snapshot_t snapshot;
    float raw_angle_deg;

    CL_GetSnapshot(MOTOR_AXIS_X, &snapshot);
    raw_angle_deg = Encoder_GetRawAngle(ENCODER_AXIS_X);

    if (s_level_default_valid != 0U) {
        Serial_Printf(
            "ANGLE relative=%.3f deg raw=%.3f deg target=%.3f deg "
            "level_default_raw=%.3f deg valid=1\r\n",
            (double)snapshot.current_angle_deg,
            (double)raw_angle_deg,
            (double)snapshot.target_angle_deg,
            (double)s_level_default_raw_deg);
    } else {
        Serial_Printf(
            "ANGLE relative=%.3f deg raw=%.3f deg target=%.3f deg "
            "level_default_raw=unset valid=0\r\n",
            (double)snapshot.current_angle_deg,
            (double)raw_angle_deg,
            (double)snapshot.target_angle_deg);
    }
}

static void Vofa_PrintMotorState(void)
{
    CL_Snapshot_t cl;
    BallBalanceSnapshot_t balance;
    float raw_angle_deg;

    CL_GetSnapshot(MOTOR_AXIS_X, &cl);
    BallBalance_GetSnapshot(&balance);
    raw_angle_deg = Encoder_GetRawAngle(ENCODER_AXIS_X);

    Serial_Printf(
        "MOTORSTATE busy=%u dir=%u remain=%lu active=%u reached=%u "
        "fault=%u count=%ld target_count=%ld error_count=%ld\r\n",
        (unsigned)Motor_IsBusy(MOTOR_AXIS_X),
        (unsigned)Motor_GetDirection(MOTOR_AXIS_X),
        (unsigned long)Motor_GetRemainingSteps(MOTOR_AXIS_X),
        (unsigned)cl.active,
        (unsigned)cl.reached,
        (unsigned)cl.fault,
        (long)cl.current_count,
        (long)cl.target_count,
        (long)cl.error_count);

    if (s_level_default_valid != 0U) {
        Serial_Printf(
            "MOTORANGLE relative=%.3f raw=%.3f target=%.3f "
            "default_raw=%.3f levelmove=%u level_calibrated=%u\r\n",
            (double)cl.current_angle_deg,
            (double)raw_angle_deg,
            (double)cl.target_angle_deg,
            (double)s_level_default_raw_deg,
            (unsigned)s_level_move_state,
            (unsigned)balance.level_calibrated);
    } else {
        Serial_Printf(
            "MOTORANGLE relative=%.3f raw=%.3f target=%.3f "
            "default_raw=unset levelmove=%u level_calibrated=%u\r\n",
            (double)cl.current_angle_deg,
            (double)raw_angle_deg,
            (double)cl.target_angle_deg,
            (unsigned)s_level_move_state,
            (unsigned)balance.level_calibrated);
    }
}

static void Vofa_PrintImu(void)
{
    ImuFeedforwardSnapshot_t snapshot;

    ImuFeedforward_GetSnapshot(&snapshot);
    Serial_Printf(
        "IMU state=%s samples=%lu errors=%lu "
        "a_g=%.4f,%.4f,%.4f gyro=%.3f,%.3f,%.3f "
        "pitch=%.3f accel=%.3f ff_angle=%.3f\r\n",
        Vofa_ImuStateName(snapshot.state),
        (unsigned long)snapshot.sample_count,
        (unsigned long)snapshot.read_error_count,
        (double)snapshot.acceleration_x_g,
        (double)snapshot.acceleration_y_g,
        (double)snapshot.acceleration_z_g,
        (double)snapshot.gyro_x_dps,
        (double)snapshot.gyro_y_dps,
        (double)snapshot.gyro_z_dps,
        (double)snapshot.pitch_deg,
        (double)snapshot.filtered_acceleration_mps2,
        (double)snapshot.feedforward_angle_deg);
}

static void Vofa_PrintCarLink(void)
{
    CarMotionRxSnapshot_t link;

    CarMotionRx_GetSnapshot(&link);
    Serial_Printf(
        "CAR_LINK1 UART1 PB7_RX/PB6_TX 115200 valid=%u active=%u "
        "running=%u pretilt=%u seq=%u flags=0x%02X cps=%d accel=%dmm/s2 "
        "filtered=%.4fm/s2 ff=%.4fdeg\r\n",
        (unsigned)link.link_valid,
        (unsigned)link.motion_active,
        (unsigned)link.running_active,
        (unsigned)link.pretilt_active,
        (unsigned)link.sequence,
        (unsigned)link.flags,
        (int)link.planned_cps,
        (int)link.acceleration_mm_s2,
        (double)link.filtered_acceleration_mps2,
        (double)link.feedforward_angle_deg);

    Serial_Printf(
        "CAR_LINK2 mission=%u pending=%u emergency=%u gain=%.3f "
        "limit=%.3f dead=%.4f dir=%d age=%lu frames=%lu checksum=%lu "
        "duplicate=%lu bytes=%lu irq=%lu poll=%lu\r\n",
        (unsigned)link.active_mission,
        (unsigned)link.pending_start_request,
        (unsigned)link.emergency_active,
        (double)link.gain,
        (double)link.angle_limit_deg,
        (double)link.deadband_mps2,
        (int)link.direction_sign,
        (unsigned long)link.frame_age_ms,
        (unsigned long)link.valid_frame_count,
        (unsigned long)link.checksum_error_count,
        (unsigned long)link.duplicate_frame_count,
        (unsigned long)link.rx_byte_count,
        (unsigned long)link.irq_count,
        (unsigned long)link.poll_drain_count);

    Serial_Printf(
        "CAR_ACK_TX mission=%u active=%u latched=%u sent=%lu dropped=%lu "
        "bytes=%lu busy=%u\r\n",
        (unsigned)s_remote_active_mission,
        (unsigned)(BallBalance_IsEnabled() != 0U),
        (unsigned)s_remote_target_latched,
        (unsigned long)link.ack_sent_frame_count,
        (unsigned long)link.ack_dropped_frame_count,
        (unsigned long)link.ack_sent_byte_count,
        (unsigned)link.ack_tx_busy);
}

static void Vofa_PrintPlot(void)
{
    if (s_plot_mode == VOFA_PLOT_BALANCE) {
        BallBalanceSnapshot_t snapshot;
        BallBalance_GetSnapshot(&snapshot);

        /* VOFA+ FireWater: pure comma-separated numbers followed by LF. */
        Serial_Printf(
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u\n",
            (double)snapshot.target_ball_cm,
            (double)snapshot.current_ball_cm,
            (double)snapshot.ball_error_cm,
            (double)snapshot.ball_velocity_cm_s,
            (double)snapshot.flap_target_deg,
            (double)snapshot.motor_target_deg,
            (double)snapshot.motor_current_deg,
            (unsigned)snapshot.state,
            (unsigned)snapshot.fault);
    } else if (s_plot_mode == VOFA_PLOT_IMU) {
        ImuFeedforwardSnapshot_t snapshot;
        ImuFeedforward_GetSnapshot(&snapshot);

        Serial_Printf(
            "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\n",
            (double)snapshot.acceleration_x_g,
            (double)snapshot.acceleration_y_g,
            (double)snapshot.acceleration_z_g,
            (double)snapshot.gyro_x_dps,
            (double)snapshot.gyro_y_dps,
            (double)snapshot.gyro_z_dps,
            (double)snapshot.pitch_deg,
            (double)snapshot.filtered_acceleration_mps2,
            (double)snapshot.feedforward_angle_deg,
            (unsigned)snapshot.state);
    } else if (s_plot_mode == VOFA_PLOT_MOTOR) {
        CL_Snapshot_t snapshot;
        CL_GetSnapshot(MOTOR_AXIS_X, &snapshot);

        Serial_Printf(
            "%.4f,%.4f,%.4f,%u,%u\n",
            (double)snapshot.target_angle_deg,
            (double)snapshot.current_angle_deg,
            (double)(snapshot.target_angle_deg -
                     snapshot.current_angle_deg),
            (unsigned)snapshot.reached,
            (unsigned)snapshot.fault);
    } else if (s_plot_mode == VOFA_PLOT_REQ4) {
        BallBalanceSnapshot_t snapshot;
        BallBalance_GetSnapshot(&snapshot);

        Serial_Printf(
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u,%u,%u,%u\n",
            (double)snapshot.target_ball_cm,
            (double)snapshot.current_ball_cm,
            (double)snapshot.ball_error_cm,
            (double)snapshot.req4_raw_velocity_cm_s,
            (double)snapshot.req4_filtered_velocity_cm_s,
            (double)snapshot.req4_predicted_error_cm,
            (double)snapshot.p_output_deg,
            (double)snapshot.i_output_deg,
            (double)snapshot.d_output_deg,
            (double)snapshot.visual_flap_target_deg,
            (double)snapshot.feedforward_acceleration_mps2,
            (double)snapshot.feedforward_angle_deg,
            (double)snapshot.flap_target_deg,
            (double)snapshot.motor_current_deg,
            (unsigned)snapshot.feedforward_valid,
            (unsigned)snapshot.req4_breakaway_active,
            (unsigned)snapshot.req4_visual_saturated,
            (unsigned)snapshot.req4_in_one_cm_band,
            (unsigned)snapshot.fault);
    }
}

static void Vofa_Test3TurnToNegative(
    uint32_t now_ms,
    const BallBalanceSnapshot_t *snapshot,
    uint8_t positive_reached)
{
    uint32_t total_ms;

    total_ms = now_ms - s_test_start_ms;

    s_test_positive_reached = positive_reached;
    if (snapshot != NULL) {
        s_test_positive_turn_cm = snapshot->current_ball_cm;
        s_test_positive_error_cm =
            fabsf(TEST3_POSITIVE_TARGET_CM - snapshot->current_ball_cm);
    }

    if (BallBalance_SetTargetCm(TEST3_NEGATIVE_TARGET_CM) == 0U) {
        Serial_SendString("ERR test3 cannot set -5 cm\r\n");
        s_test_state = TEST3_FINISHED;
        return;
    }

    s_test_state = TEST3_TO_NEGATIVE;
    s_test_stage_ms = now_ms;
    Serial_Printf(
        "TEST3 turn: ball_center=%.3f cm, +5_center_error=%.3f cm, t=%lu ms, result=%s\r\n",
        (double)s_test_positive_turn_cm,
        (double)s_test_positive_error_cm,
        (unsigned long)total_ms,
        (positive_reached != 0U) ? "POS_PASS" : "POS_TIMEOUT");
}

static uint8_t Vofa_CameraPositionValid(uint32_t now_ms)
{
    BallBalanceSnapshot_t snapshot;
    BallBalanceConfig_t config;

    BallBalance_GetSnapshot(&snapshot);
    BallBalance_GetConfig(&config);

    return (snapshot.measurement_sequence != 0U) &&
        ((uint32_t)(now_ms - snapshot.measurement_timestamp_ms) <=
         config.camera_timeout_ms) &&
        (snapshot.fault != BALL_BALANCE_FAULT_CAMERA_TIMEOUT);
}

static void Vofa_ClearRemoteMission(void)
{
    s_remote_pending_mission = CAR_MOTION_MISSION_NONE;
    s_remote_active_mission = CAR_MOTION_MISSION_NONE;
    s_remote_target_latched = 0U;
    s_remote_captured_target_cm = 0.0f;
}

static uint8_t Vofa_StartContinuousBalance(
    uint32_t now_ms,
    uint8_t mission,
    float target_cm,
    uint8_t remote_start)
{
    if (BallBalance_IsReady() == 0U ||
        Vofa_CameraPositionValid(now_ms) == 0U) {
        return 0U;
    }

    Vofa_CancelMotorTest();
    Vofa_CancelLevelMove();
    s_test_state = TEST3_IDLE;

    BallBalance_SetRequirement4Mode(0U);
    BallBalance_SetExternalFeedforwardEnabled(0U);
    BallBalance_ClearFault();

    if (BallBalance_SetTargetCm(target_cm) == 0U ||
        BallBalance_Enable(1U) == 0U) {
        return 0U;
    }

    /* Enable after BallBalance_Enable(), because Enable resets controller state. */
    BallBalance_SetRequirement4Mode(1U);
    BallBalance_SetExternalFeedforwardEnabled(1U);

    if (remote_start != 0U) {
        s_remote_active_mission = mission;
        s_remote_target_latched = 1U;
    } else {
        Vofa_ClearRemoteMission();
    }

    return 1U;
}

static void Vofa_StartRequirement4(uint32_t now_ms)
{
    if (Vofa_StartContinuousBalance(
            now_ms,
            CAR_MOTION_MISSION_CENTER,
            0.0f,
            0U) == 0U) {
        Serial_SendString(
            "ERR req45 requires level zero and a valid camera ball position\r\n");
        return;
    }

    Serial_SendString(
        "REQ45 start: target ball center=0 cm, visual feedback + car "
        "acceleration feed-forward; plot remains manual\r\n");
}

static void Vofa_StartRequirement4VisualOnly(uint32_t now_ms)
{
    Vofa_StartRequirement4(now_ms);
    if (BallBalance_IsRequirement4Mode() != 0U) {
        BallBalance_SetExternalFeedforwardEnabled(0U);
        Serial_SendString(
            "REQ45 visual-only: car acceleration feed-forward disabled\r\n");
    }
}

static void Vofa_StartRequirement6(uint32_t now_ms)
{
    BallBalanceSnapshot_t snapshot;

    BallBalance_GetSnapshot(&snapshot);
    if (Vofa_StartContinuousBalance(
            now_ms,
            CAR_MOTION_MISSION_CAPTURED,
            snapshot.current_ball_cm,
            0U) == 0U) {
        Serial_SendString(
            "ERR req6 requires level zero and a valid current camera position\r\n");
        return;
    }

    Serial_Printf(
        "REQ6 start: captured target=%.3f cm; same visual + acceleration control\r\n",
        (double)snapshot.current_ball_cm);
}

static void Vofa_ProcessCarMission(uint32_t now_ms)
{
    uint8_t request = CarMotionRx_TakeStartRequest();

    if (request != CAR_MOTION_MISSION_NONE) {
        s_remote_pending_mission = request;
        s_remote_active_mission = CAR_MOTION_MISSION_NONE;
        s_remote_target_latched = 0U;
        s_remote_captured_target_cm = 0.0f;
        s_remote_last_retry_ms = now_ms - 20U;

        if (request == CAR_MOTION_MISSION_CENTER) {
            s_remote_captured_target_cm = 0.0f;
            s_remote_target_latched = 1U;
        } else if (request == CAR_MOTION_MISSION_CAPTURED &&
                   Vofa_CameraPositionValid(now_ms) != 0U) {
            BallBalanceSnapshot_t snapshot;
            BallBalance_GetSnapshot(&snapshot);
            s_remote_captured_target_cm = snapshot.current_ball_cm;
            s_remote_target_latched = 1U;
        }

        Serial_Printf(
            "CAR_START request mission=%u target_latched=%u target=%.3fcm; "
            "waiting for level/camera/controller ready\r\n",
            (unsigned)request,
            (unsigned)s_remote_target_latched,
            (double)s_remote_captured_target_cm);
    }

    if (CarMotionRx_IsEmergency(now_ms) != 0U) {
        if (s_remote_pending_mission != CAR_MOTION_MISSION_NONE ||
            s_remote_active_mission != CAR_MOTION_MISSION_NONE) {
            BallBalance_Stop();
            Vofa_ClearRemoteMission();
            Serial_SendString(
                "CAR emergency/stop received: balance controller stopped\r\n");
        }
        return;
    }

    if (s_remote_pending_mission == CAR_MOTION_MISSION_NONE) {
        return;
    }

    if ((uint32_t)(now_ms - s_remote_last_retry_ms) < 20U) {
        return;
    }
    s_remote_last_retry_ms = now_ms;

    if (s_remote_pending_mission == CAR_MOTION_MISSION_CAPTURED &&
        s_remote_target_latched == 0U &&
        Vofa_CameraPositionValid(now_ms) != 0U) {
        BallBalanceSnapshot_t snapshot;
        BallBalance_GetSnapshot(&snapshot);
        s_remote_captured_target_cm = snapshot.current_ball_cm;
        s_remote_target_latched = 1U;
        Serial_Printf(
            "CAR_REQ6 target captured at first valid camera sample: %.3f cm\r\n",
            (double)s_remote_captured_target_cm);
    }

    if (BallBalance_IsReady() == 0U ||
        Vofa_CameraPositionValid(now_ms) == 0U ||
        s_remote_target_latched == 0U) {
        return;
    }

    if (s_remote_pending_mission == CAR_MOTION_MISSION_CENTER) {
        if (Vofa_StartContinuousBalance(
                now_ms,
                CAR_MOTION_MISSION_CENTER,
                0.0f,
                1U) != 0U) {
            s_remote_pending_mission = CAR_MOTION_MISSION_NONE;
            Serial_SendString(
                "CAR_REQ45 accepted: target=0 cm; READY ACK enabled\r\n");
        }
    } else if (s_remote_pending_mission == CAR_MOTION_MISSION_CAPTURED) {
        if (Vofa_StartContinuousBalance(
                now_ms,
                CAR_MOTION_MISSION_CAPTURED,
                s_remote_captured_target_cm,
                1U) != 0U) {
            s_remote_pending_mission = CAR_MOTION_MISSION_NONE;
            Serial_Printf(
                "CAR_REQ6 accepted: captured ball-center target=%.3f cm; "
                "READY ACK enabled\r\n",
                (double)s_remote_captured_target_cm);
        }
    } else {
        s_remote_pending_mission = CAR_MOTION_MISSION_NONE;
    }
}

static void Vofa_UpdateCarAck(uint32_t now_ms)
{
    BallBalanceSnapshot_t snapshot;
    uint8_t mission;
    uint8_t camera_valid;
    uint8_t control_active;
    uint8_t pretilt_ready = 0U;

    BallBalance_GetSnapshot(&snapshot);
    mission = (s_remote_pending_mission != CAR_MOTION_MISSION_NONE) ?
        s_remote_pending_mission : s_remote_active_mission;
    camera_valid = Vofa_CameraPositionValid(now_ms);
    control_active =
        (s_remote_active_mission != CAR_MOTION_MISSION_NONE) &&
        (snapshot.requirement4_mode != 0U) &&
        (snapshot.enabled != 0U) &&
        (snapshot.fault == BALL_BALANCE_FAULT_NONE);

    /*
     * The car may start only after the balance board has received PRETILT,
     * calculated a non-zero launch feed-forward and physically moved the
     * screw close to that target. This prevents a fixed delay from expiring
     * while the actuator is still travelling.
     */
    if (CarMotionRx_IsPretiltActive(now_ms) != 0U &&
        control_active != 0U &&
        snapshot.feedforward_enabled != 0U &&
        snapshot.feedforward_valid != 0U &&
        fabsf(snapshot.feedforward_angle_deg) >=
            VOFA_PRETILT_MIN_FF_ANGLE_DEG &&
        fabsf(snapshot.motor_target_deg - snapshot.motor_current_deg) <=
            VOFA_PRETILT_MOTOR_WINDOW_DEG) {
        pretilt_ready = 1U;
    }

    CarMotionRx_SetLocalMissionStatus(
        mission,
        snapshot.level_calibrated,
        camera_valid,
        control_active,
        s_remote_target_latched,
        pretilt_ready,
        (uint8_t)snapshot.fault);
}

static void Vofa_StartTest3(uint32_t now_ms)
{
    BallBalanceConfig_t config;

    if (BallBalance_IsReady() == 0U) {
        Serial_SendString("ERR test3 requires level zero and a valid camera/sample position\r\n");
        return;
    }

    /* Reject an accidental run with parameters that do not match this rig. */
    BallBalance_GetConfig(&config);
    if (config.control_sign != -1 ||
        config.lever_arm_mm < 234.0f || config.lever_arm_mm > 236.0f ||
        config.max_flap_angle_deg < 1.30f) {
        Serial_SendString(
            "ERR test3 config: use sign=-1, arm=235, maxangle=1.50 recommended\r\n");
        return;
    }

    Vofa_CancelMotorTest();
    Vofa_CancelLevelMove();
    Vofa_ClearRemoteMission();
    /* Requirement 3 is intentionally kept on its original visual-only path. */
    BallBalance_SetRequirement4Mode(0U);
    BallBalance_SetExternalFeedforwardEnabled(0U);
    BallBalance_ClearFault();
    if (BallBalance_SetTargetCm(TEST3_POSITIVE_TARGET_CM) == 0U ||
        BallBalance_Enable(1U) == 0U) {
        Serial_SendString("ERR cannot start test3\r\n");
        return;
    }

    /* Requirement-3 logs need enough time resolution to see the turn. */
    s_plot_mode = VOFA_PLOT_BALANCE;
    s_plot_period_ms = 50U;

    s_test_state = TEST3_TO_POSITIVE;
    s_test_start_ms = now_ms;
    s_test_stage_ms = now_ms;
    s_test_positive_confirm_ms = now_ms;
    s_test_positive_confirm_active = 0U;
    s_test_positive_reached = 0U;
    s_test_positive_turn_cm = 0.0f;
    s_test_positive_error_cm = 100.0f;

    Serial_SendString(
        "TEST3 start: BALL CENTER O -> +5 cm -> -5 cm; limit=5000 ms, center_error<=0.5 cm, motor_top=9kHz, maxangle=1.50 recommended, pos_band=4.50..5.50, pos_level=1.15s/0.12deg, neg_drive=1.35->0.28 deg, stall_boost=0.42 deg, speed_cap=6.40..7.40, neg_predict=0.24 s, brake=0.80->0.55 deg, release=0.05 s\r\n");
}

static void Vofa_ProcessTest3(uint32_t now_ms)
{
    BallBalanceSnapshot_t snapshot;
    uint32_t total_ms;
    uint32_t stage_ms;
    float positive_error_cm;
    float negative_error_cm;
    uint8_t pass_position;
    uint8_t pass_time;

    if (s_test_state == TEST3_IDLE || s_test_state == TEST3_FINISHED) {
        return;
    }

    BallBalance_GetSnapshot(&snapshot);
    total_ms = now_ms - s_test_start_ms;
    stage_ms = now_ms - s_test_stage_ms;

    if (snapshot.fault != BALL_BALANCE_FAULT_NONE) {
        s_test_state = TEST3_FINISHED;
        Serial_Printf(
            "TEST3 aborted: fault=%u, total=%lu ms\r\n",
            (unsigned)snapshot.fault,
            (unsigned long)total_ms);
        return;
    }

    if (s_test_state == TEST3_TO_POSITIVE) {
        positive_error_cm =
            fabsf(TEST3_POSITIVE_TARGET_CM - snapshot.current_ball_cm);

        /*
         * The camera coordinate is the BALL CENTER coordinate.  Turn only
         * after that center enters 4.50..5.50 cm; no ball-radius offset is
         * applied.  The one-sided approach normally enters through 4.55 cm.
         */
        if (snapshot.current_ball_cm >= TEST3_POSITIVE_TURN_MIN_CM &&
            snapshot.current_ball_cm <= TEST3_POSITIVE_TURN_MAX_CM) {
#if (TEST3_POSITIVE_CONFIRM_MS == 0U)
            Vofa_Test3TurnToNegative(now_ms, &snapshot, 1U);
#else
            if (s_test_positive_confirm_active == 0U) {
                s_test_positive_confirm_active = 1U;
                s_test_positive_confirm_ms = now_ms;
            }

            if ((uint32_t)(now_ms - s_test_positive_confirm_ms) >=
                    TEST3_POSITIVE_CONFIRM_MS) {
                Vofa_Test3TurnToNegative(now_ms, &snapshot, 1U);
            }
#endif
        } else {
            s_test_positive_confirm_active = 0U;
        }

        if (s_test_state == TEST3_TO_POSITIVE &&
            stage_ms >= TEST3_POSITIVE_STAGE_LIMIT_MS) {
            s_test_state = TEST3_FINISHED;
            BallBalance_Stop();
            s_plot_mode = VOFA_PLOT_OFF;
            Serial_Printf(
                "TEST3 POS_TIMEOUT: ball_center=%.3f cm, +5_center_error=%.3f cm, "
                "t=%lu ms; sequence aborted, plot stopped\r\n",
                (double)snapshot.current_ball_cm,
                (double)positive_error_cm,
                (unsigned long)total_ms);
        }
        return;
    }

    if (s_test_state == TEST3_TO_NEGATIVE) {
        negative_error_cm =
            fabsf(TEST3_NEGATIVE_TARGET_CM - snapshot.current_ball_cm);

        if (BallBalance_IsSettled() != 0U) {
            pass_position =
                (s_test_positive_reached != 0U &&
                 s_test_positive_error_cm <= TEST3_POSITION_LIMIT_CM &&
                 negative_error_cm <= TEST3_POSITION_LIMIT_CM) ? 1U : 0U;
            pass_time = (total_ms <= TEST3_TOTAL_LIMIT_MS) ? 1U : 0U;
            s_test_state = TEST3_FINISHED;

            Serial_Printf(
                "TEST3 finished: total=%lu ms, +turn_center=%.3f cm(err=%.3f), "
                "final_center=%.3f cm(err=%.3f), result=%s\r\n",
                (unsigned long)total_ms,
                (double)s_test_positive_turn_cm,
                (double)s_test_positive_error_cm,
                (double)snapshot.current_ball_cm,
                (double)negative_error_cm,
                (pass_time != 0U && pass_position != 0U) ?
                    "PASS" : ((pass_time == 0U) ? "TIME_OVER" : "POSITION_FAIL"));
        } else if (total_ms > TEST3_TOTAL_LIMIT_MS) {
            s_test_state = TEST3_FINISHED;
            Serial_Printf(
                "TEST3 timeout: total=%lu ms, +turn_center=%.3f(err=%.3f), "
                "current_center=%.3f(err=%.3f); keep holding -5 cm\r\n",
                (unsigned long)total_ms,
                (double)s_test_positive_turn_cm,
                (double)s_test_positive_error_cm,
                (double)snapshot.current_ball_cm,
                (double)negative_error_cm);
        }
    }
}


static void Vofa_CancelMotorTest(void)
{
    s_motor_test_state = MOTOR_TEST_IDLE;
}

static void Vofa_PrepareManualMotor(void)
{
    /*
     * The ball controller and manual motor commands must never write the
     * closed-loop target at the same time.
     */
    BallBalance_Stop();
    s_test_state = TEST3_IDLE;
    Vofa_ClearRemoteMission();
    Vofa_CancelMotorTest();
    Vofa_CancelLevelMove();
}

static void Vofa_StartMotorTest(uint32_t now_ms)
{
    (void)now_ms;

    if (s_motor_test_state != MOTOR_TEST_IDLE) {
        Serial_SendString("ERR motor test is already running\r\n");
        return;
    }

    BallBalance_Stop();
    s_test_state = TEST3_IDLE;
    Vofa_CancelLevelMove();

    if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
        Serial_SendString("ERR clear the motor fault before mtest\r\n");
        return;
    }

    if (CL_SetTargetAngle(
            MOTOR_AXIS_X,
            MOTOR_TEST_NEGATIVE_DEG) != MOTOR_OK) {
        Serial_SendString("ERR cannot set motor test start target\r\n");
        return;
    }

    s_motor_test_state = MOTOR_TEST_TO_NEGATIVE;
    Serial_SendString(
        "MTEST moving to -900 deg; verify the software zero is at travel center\r\n");
}

static void Vofa_ProcessMotorTest(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (s_motor_test_state == MOTOR_TEST_IDLE) {
        return;
    }

    if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
        CL_StopAll();
        s_motor_test_state = MOTOR_TEST_IDLE;
        Serial_Printf(
            "MTEST stopped, motor fault=%u\r\n",
            (unsigned)CL_GetFault(MOTOR_AXIS_X));
        return;
    }

    switch (s_motor_test_state) {
        case MOTOR_TEST_TO_NEGATIVE:
            if (CL_IsReached(MOTOR_AXIS_X) != 0U) {
                s_motor_test_wait_ms = now_ms;
                s_motor_test_state = MOTOR_TEST_WAIT_NEGATIVE;
                Serial_SendString("MTEST reached -900 deg; settling\r\n");
            }
            break;

        case MOTOR_TEST_WAIT_NEGATIVE:
            if ((uint32_t)(now_ms - s_motor_test_wait_ms) >=
                MOTOR_TEST_SETTLE_MS) {
                s_motor_test_start_ms = now_ms;
                if (CL_SetTargetAngle(
                        MOTOR_AXIS_X,
                        MOTOR_TEST_POSITIVE_DEG) != MOTOR_OK) {
                    s_motor_test_state = MOTOR_TEST_IDLE;
                    Serial_SendString("ERR cannot set +900 deg target\r\n");
                    return;
                }
                s_motor_test_state = MOTOR_TEST_TO_POSITIVE;
                Serial_SendString("MTEST timing -900 deg -> +900 deg\r\n");
            }
            break;

        case MOTOR_TEST_TO_POSITIVE:
            if (CL_IsReached(MOTOR_AXIS_X) != 0U) {
                elapsed_ms = now_ms - s_motor_test_start_ms;
                s_motor_test_state = MOTOR_TEST_IDLE;
                Serial_Printf(
                    "MTEST complete, elapsed=%lu ms (%.3f s)\r\n",
                    (unsigned long)elapsed_ms,
                    (double)elapsed_ms / 1000.0);
            }
            break;

        default:
            s_motor_test_state = MOTOR_TEST_IDLE;
            break;
    }
}

static void Vofa_CancelLevelMove(void)
{
    s_level_move_state = LEVEL_MOVE_IDLE;
}

static void Vofa_SaveLevelHere(void)
{
    BallBalance_Stop();
    s_test_state = TEST3_IDLE;
    Vofa_ClearRemoteMission();
    Vofa_CancelMotorTest();
    Vofa_CancelLevelMove();

    s_level_default_raw_deg = Encoder_GetRawAngle(ENCODER_AXIS_X);
    s_level_default_valid = 1U;
    BallBalance_SetLevelZero();

    Serial_Printf(
        "OK level saved: raw=%.3f deg; current position is now level zero\r\n",
        (double)s_level_default_raw_deg);
}

static void Vofa_StartLevelMove(uint32_t now_ms)
{
    float current_raw_deg;
    float current_relative_deg;
    float target_relative_deg;

    (void)now_ms;

    if (s_level_default_valid == 0U) {
        /*
         * Safe fallback for projects built with an old/blank configuration:
         * assume the present power-up reference is the level position. This is
         * equivalent to the previously required "levelsave; levelgo" sequence.
         */
        s_level_default_raw_deg = Encoder_GetRawAngle(ENCODER_AXIS_X);
        s_level_default_valid = 1U;
        Serial_Printf(
            "WARN no saved level; using current raw=%.3f deg for this session\r\n",
            (double)s_level_default_raw_deg);
    }

    BallBalance_Stop();
    s_test_state = TEST3_IDLE;
    Vofa_CancelMotorTest();
    Vofa_CancelLevelMove();

    if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
        Serial_SendString("ERR clear motor fault before levelgo\r\n");
        return;
    }

    current_raw_deg = Encoder_GetRawAngle(ENCODER_AXIS_X);
    current_relative_deg = CL_GetCurrentAngle(MOTOR_AXIS_X);
    target_relative_deg = current_relative_deg +
        (s_level_default_raw_deg - current_raw_deg);

    if (target_relative_deg < -VOFA_LEVEL_MOVE_LIMIT_DEG ||
        target_relative_deg > VOFA_LEVEL_MOVE_LIMIT_DEG ||
        CL_SetTargetAngle(MOTOR_AXIS_X, target_relative_deg) != MOTOR_OK) {
        Serial_Printf(
            "ERR levelgo target %.3f deg is outside safe relative travel\r\n",
            (double)target_relative_deg);
        return;
    }

    s_level_move_state = LEVEL_MOVE_TO_DEFAULT;
    Serial_Printf(
        "LEVELGO raw_now=%.3f raw_target=%.3f relative_target=%.3f deg\r\n",
        (double)current_raw_deg,
        (double)s_level_default_raw_deg,
        (double)target_relative_deg);
}

static void Vofa_ProcessLevelMove(uint32_t now_ms)
{
    if (s_level_move_state == LEVEL_MOVE_IDLE) {
        return;
    }

    if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
        s_level_move_state = LEVEL_MOVE_IDLE;
        Serial_Printf(
            "LEVELGO stopped, motor fault=%u\r\n",
            (unsigned)CL_GetFault(MOTOR_AXIS_X));
        return;
    }

    if (s_level_move_state == LEVEL_MOVE_TO_DEFAULT) {
        if (CL_IsReached(MOTOR_AXIS_X) != 0U) {
            s_level_move_settle_ms = now_ms;
            s_level_move_state = LEVEL_MOVE_SETTLING;
        }
        return;
    }

    if (s_level_move_state == LEVEL_MOVE_SETTLING &&
        (uint32_t)(now_ms - s_level_move_settle_ms) >=
            VOFA_LEVEL_MOVE_SETTLE_MS) {
        float reached_raw_deg = Encoder_GetRawAngle(ENCODER_AXIS_X);
        float error_deg = s_level_default_raw_deg - reached_raw_deg;

        BallBalance_SetLevelZero();
        s_level_move_state = LEVEL_MOVE_IDLE;
        Serial_Printf(
            "OK LEVELGO complete: raw=%.3f deg error=%.3f deg; level zero set\r\n",
            (double)reached_raw_deg,
            (double)error_deg);
    }
}

static uint8_t Vofa_UpdateBalanceFloat(
    const char *name,
    const char *value_text)
{
    BallBalanceConfig_t config;
    float value;

    if (Vofa_ParseFloat(value_text, &value) == 0U) {
        return 0U;
    }

    BallBalance_GetConfig(&config);

    if (strcmp(name, "kp") == 0) {
        config.kp = value;
    } else if (strcmp(name, "ki") == 0) {
        config.ki = value;
    } else if (strcmp(name, "kd") == 0) {
        config.kd = value;
    } else if (strcmp(name, "arm") == 0) {
        config.lever_arm_mm = value;
    } else if (strcmp(name, "maxangle") == 0) {
        config.max_flap_angle_deg = value;
    } else if (strcmp(name, "travel") == 0) {
        if (value < 0.0f) {
            value = -value;
        }
        config.actuator_min_mm = -value;
        config.actuator_max_mm = value;
    } else {
        return 0U;
    }

    return BallBalance_SetConfig(&config);
}

static void Vofa_ProcessCommand(char *raw_command, uint32_t now_ms)
{
    char *command = Vofa_Normalize(raw_command);
    char *value_text;
    float float_value;
    uint32_t u32_value;
    BallBalanceConfig_t config;

    if (command == NULL || *command == '\0') {
        return;
    }

    if (strcmp(command, "help") == 0) {
        Vofa_PrintHelp();
    } else if (strcmp(command, "ping") == 0) {
        Serial_SendString("PONG UART0 PB0(TX) PB1(RX) 115200 8N1\r\n");
    } else if (strcmp(command, "serial") == 0) {
        Vofa_PrintSerial();
    } else if (strcmp(command, "status") == 0 || strcmp(command, "get") == 0) {
        Vofa_PrintStatus();
    } else if (strcmp(command, "config") == 0) {
        Vofa_PrintConfig();
    } else if (strcmp(command, "camera") == 0) {
        Vofa_PrintCamera();
    } else if (strcmp(command, "camping") == 0) {
        if (CameraUart_SendString("$PING*") != 0U) {
            Serial_SendString("OK sent $PING* on UART2 PB15(TX); run camera again after 200 ms\r\n");
        } else {
            Serial_SendString("ERR UART2 camera TX timeout\r\n");
        }
    } else if (strcmp(command, "motor") == 0) {
        Vofa_PrintMotor();
    } else if (strcmp(command, "mangle") == 0 ||
               strcmp(command, "angle") == 0) {
        Vofa_PrintMotorAngle();
    } else if (strcmp(command, "motorstate") == 0 ||
               strcmp(command, "mstate") == 0 ||
               strcmp(command, "levelstatus") == 0) {
        Vofa_PrintMotorState();
    } else if (strcmp(command, "levelsave") == 0) {
        Vofa_SaveLevelHere();
    } else if (strcmp(command, "levelgo") == 0) {
        Vofa_StartLevelMove(now_ms);
    } else if (strcmp(command, "imu") == 0) {
        Vofa_PrintImu();
    } else if (strcmp(command, "carlink") == 0 ||
               strcmp(command, "carffconfig") == 0) {
        Vofa_PrintCarLink();
    } else if (strcmp(command, "req4status") == 0 ||
               strcmp(command, "r4status") == 0) {
        Vofa_PrintReq4Status();
    } else if (strcmp(command, "req4config") == 0 ||
               strcmp(command, "r4config") == 0) {
        Vofa_PrintReq4Config();
    } else if (strcmp(command, "r4reset") == 0) {
        Req4VisualControl_Reset();
        Serial_SendString("OK req4 visual state reset\r\n");
    } else if (strcmp(command, "r4defaults") == 0) {
        Req4VisualControl_RestoreDefaults();
        Req4VisualControl_Reset();
        Serial_SendString("OK req4 defaults restored\r\n");
        Vofa_PrintReq4Config();
    } else if (strcmp(command, "req4visual") == 0 ||
               strcmp(command, "centeronly") == 0) {
        Vofa_StartRequirement4VisualOnly(now_ms);
    } else if (strcmp(command, "req45") == 0 ||
               strcmp(command, "req4") == 0 ||
               strcmp(command, "centerff") == 0) {
        Vofa_StartRequirement4(now_ms);
    } else if (strcmp(command, "req6") == 0 ||
               strcmp(command, "captureff") == 0) {
        Vofa_StartRequirement6(now_ms);
    } else if (strcmp(command, "carreset") == 0) {
        CarMotionRx_ResetDiagnostics();
        Serial_SendString("OK car motion UART diagnostics reset\r\n");
    } else if (strcmp(command, "level") == 0 ||
               strcmp(command, "mzero") == 0) {
        s_test_state = TEST3_IDLE;
        Vofa_ClearRemoteMission();
        Vofa_CancelMotorTest();
        Vofa_CancelLevelMove();
        BallBalance_SetLevelZero();
        Serial_SendString("OK level/motor zero set at current mechanical position\r\n");
    } else if (strcmp(command, "clear") == 0 ||
               strcmp(command, "mclear") == 0) {
        s_test_state = TEST3_IDLE;
        Vofa_CancelMotorTest();
        Vofa_CancelLevelMove();
        BallBalance_ClearFault();
        Serial_SendString("OK balance and motor faults cleared\r\n");
    } else if (strcmp(command, "stop") == 0 ||
               strcmp(command, "mstop") == 0) {
        BallBalance_Stop();
        CL_StopAll();
        s_test_state = TEST3_IDLE;
        Vofa_ClearRemoteMission();
        Vofa_CancelMotorTest();
        Vofa_CancelLevelMove();
        Serial_SendString("OK stopped\r\n");
    } else if (strcmp(command, "mdir") == 0) {
        Vofa_PrepareManualMotor();
        if (CL_TogglePositiveDirLevel(MOTOR_AXIS_X) == MOTOR_OK) {
            Serial_SendString("OK motor positive direction level toggled\r\n");
        } else {
            Serial_SendString("ERR motor must be stopped before mdir\r\n");
        }
    } else if (strcmp(command, "mtest") == 0 ||
               strcmp(command, "traveltest") == 0) {
        Vofa_StartMotorTest(now_ms);
    } else if (strcmp(command, "test") == 0 || strcmp(command, "test3") == 0) {
        Vofa_StartTest3(now_ms);
    } else if (strcmp(command, "imucal") == 0) {
        ImuFeedforward_StartCalibration();
        Serial_SendString("OK MPU6050 calibration started; keep the car still and level\r\n");
    } else if (strcmp(command, "camreset") == 0) {
        BallLink_Reset();
        CameraUart_ResetDiagnostics();
        Serial_SendString("OK camera UART and parser diagnostics reset\r\n");
    } else if (Vofa_ParseAssignment(command, "leveldefault", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            float_value < -VOFA_LEVEL_MOVE_LIMIT_DEG ||
            float_value > VOFA_LEVEL_MOVE_LIMIT_DEG) {
            Serial_SendString("ERR leveldefault range is -1700..1700 raw deg\r\n");
        } else {
            s_level_default_raw_deg = float_value;
            s_level_default_valid = 1U;
            Serial_Printf(
                "OK leveldefault_raw=%.3f deg; send levelgo to move and zero\r\n",
                (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "levelto", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            float_value < -VOFA_LEVEL_MOVE_LIMIT_DEG ||
            float_value > VOFA_LEVEL_MOVE_LIMIT_DEG) {
            Serial_SendString("ERR levelto range is -1700..1700 raw deg\r\n");
        } else {
            s_level_default_raw_deg = float_value;
            s_level_default_valid = 1U;
            Vofa_StartLevelMove(now_ms);
        }
    } else if (Vofa_ParseAssignment(command, "mtarget", &value_text) != 0U ||
               Vofa_ParseAssignment(command, "motor_target", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            float_value < -MOTOR_DEBUG_LIMIT_DEG ||
            float_value > MOTOR_DEBUG_LIMIT_DEG) {
            Serial_SendString("ERR mtarget range is -900..900 deg\r\n");
        } else {
            Vofa_PrepareManualMotor();
            if (CL_SetTargetAngle(MOTOR_AXIS_X, float_value) != MOTOR_OK) {
                Serial_SendString("ERR cannot set manual motor target\r\n");
            } else {
                Serial_Printf("OK mtarget=%.3f deg\r\n", (double)float_value);
            }
        }
    } else if (Vofa_ParseAssignment(command, "enable", &value_text) != 0U) {
        Vofa_CancelMotorTest();
        if (Vofa_ParseU32(value_text, &u32_value) == 0U || u32_value > 1U ||
            BallBalance_Enable((uint8_t)u32_value) == 0U) {
            Serial_SendString("ERR enable must be 0 or 1 and controller must be ready\r\n");
        } else {
            Serial_Printf("OK enable=%lu\r\n", (unsigned long)u32_value);
        }
    } else if (Vofa_ParseAssignment(command, "target", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            BallBalance_SetTargetCm(float_value) == 0U) {
            Serial_SendString("ERR invalid target\r\n");
        } else {
            Serial_Printf("OK target=%.3f cm\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "sample", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            BallBalance_PushBallPosition(float_value, now_ms) == 0U) {
            Serial_SendString("ERR invalid sample\r\n");
        } else {
            Serial_Printf("OK sample=%.3f cm\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "sign", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            (float_value != 1.0f && float_value != -1.0f)) {
            Serial_SendString("ERR sign must be 1 or -1\r\n");
        } else {
            BallBalance_GetConfig(&config);
            config.control_sign = (float_value < 0.0f) ? -1 : 1;
            if (BallBalance_SetConfig(&config) == 0U) {
                Serial_SendString("ERR invalid sign config\r\n");
            } else {
                Serial_Printf("OK sign=%d\r\n", (int)config.control_sign);
            }
        }
    } else if (Vofa_ParseAssignment(command, "timeout", &value_text) != 0U) {
        if (Vofa_ParseU32(value_text, &u32_value) == 0U) {
            Serial_SendString("ERR invalid timeout\r\n");
        } else {
            BallBalance_GetConfig(&config);
            config.camera_timeout_ms = u32_value;
            if (BallBalance_SetConfig(&config) == 0U) {
                Serial_SendString("ERR timeout must be at least 20 ms\r\n");
            } else {
                Serial_Printf("OK timeout=%lu ms\r\n", (unsigned long)u32_value);
            }
        }
    } else if (Vofa_ParseAssignment(command, "plot", &value_text) != 0U) {
        if (Vofa_ParseU32(value_text, &u32_value) == 0U || u32_value > 4U) {
            Serial_SendString("ERR plot must be 0, 1, 2, 3 or 4\r\n");
        } else {
            s_plot_mode = (VofaPlotMode_t)u32_value;
            Serial_Printf("OK plot=%lu\r\n", (unsigned long)u32_value);
        }
    } else if (Vofa_ParseAssignment(command, "period", &value_text) != 0U ||
               Vofa_ParseAssignment(command, "plotperiod", &value_text) != 0U) {
        if (Vofa_ParseU32(value_text, &u32_value) == 0U ||
            u32_value < VOFA_PLOT_MIN_MS || u32_value > VOFA_PLOT_MAX_MS) {
            Serial_SendString("ERR period range is 20..1000 ms\r\n");
        } else {
            s_plot_period_ms = u32_value;
            Serial_Printf("OK period=%lu ms\r\n", (unsigned long)u32_value);
        }
    } else if (Vofa_ParseAssignment(command, "r4kp", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4kp", value_text) == 0U) Serial_SendString("ERR invalid r4kp\r\n");
        else Serial_SendString("OK r4kp\r\n");
    } else if (Vofa_ParseAssignment(command, "r4ki", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4ki", value_text) == 0U) Serial_SendString("ERR invalid r4ki\r\n");
        else Serial_SendString("OK r4ki\r\n");
    } else if (Vofa_ParseAssignment(command, "r4kd", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4kd", value_text) == 0U) Serial_SendString("ERR invalid r4kd\r\n");
        else Serial_SendString("OK r4kd\r\n");
    } else if (Vofa_ParseAssignment(command, "r4predict", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4predict", value_text) == 0U) Serial_SendString("ERR r4predict range 0..0.50\r\n");
        else Serial_SendString("OK r4predict\r\n");
    } else if (Vofa_ParseAssignment(command, "r4predlimit", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4predlimit", value_text) == 0U) Serial_SendString("ERR r4predlimit range 0..3\r\n");
        else Serial_SendString("OK r4predlimit\r\n");
    } else if (Vofa_ParseAssignment(command, "r4vfilter", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4vfilter", value_text) == 0U) Serial_SendString("ERR r4vfilter range >0..1\r\n");
        else Serial_SendString("OK r4vfilter\r\n");
    } else if (Vofa_ParseAssignment(command, "r4max", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4max", value_text) == 0U) Serial_SendString("ERR r4max range >0..1.50 and >= break angle\r\n");
        else Serial_SendString("OK r4max\r\n");
    } else if (Vofa_ParseAssignment(command, "r4ilim", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4ilim", value_text) == 0U) Serial_SendString("ERR r4ilim range 0..1\r\n");
        else Serial_SendString("OK r4ilim\r\n");
    } else if (Vofa_ParseAssignment(command, "r4slew", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4slew", value_text) == 0U) Serial_SendString("ERR r4slew range >0..50\r\n");
        else Serial_SendString("OK r4slew\r\n");
    } else if (Vofa_ParseAssignment(command, "r4dead", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4dead", value_text) == 0U) Serial_SendString("ERR r4dead range 0..1\r\n");
        else Serial_SendString("OK r4dead\r\n");
    } else if (Vofa_ParseAssignment(command, "r4holdband", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4holdband", value_text) == 0U) Serial_SendString("ERR r4holdband range r4dead..2cm\r\n");
        else Serial_SendString("OK r4holdband\r\n");
    } else if (Vofa_ParseAssignment(command, "r4breakerr", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4breakerr", value_text) == 0U) Serial_SendString("ERR r4breakerr range 0..3\r\n");
        else Serial_SendString("OK r4breakerr\r\n");
    } else if (Vofa_ParseAssignment(command, "r4breakspeed", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4breakspeed", value_text) == 0U) Serial_SendString("ERR r4breakspeed range 0..5\r\n");
        else Serial_SendString("OK r4breakspeed\r\n");
    } else if (Vofa_ParseAssignment(command, "r4breakangle", &value_text) != 0U) {
        if (Vofa_UpdateReq4Float("r4breakangle", value_text) == 0U) Serial_SendString("ERR r4breakangle invalid or above r4max\r\n");
        else Serial_SendString("OK r4breakangle\r\n");
    } else if (Vofa_ParseAssignment(command, "carff", &value_text) != 0U) {
        if (Vofa_ParseU32(value_text, &u32_value) == 0U || u32_value > 1U) {
            Serial_SendString("ERR carff must be 0 or 1\r\n");
        } else {
            BallBalance_SetExternalFeedforwardEnabled((uint8_t)u32_value);
            Serial_Printf("OK carff=%lu\r\n", (unsigned long)u32_value);
        }
    } else if (Vofa_ParseAssignment(command, "carffgain", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U) {
            Serial_SendString("ERR invalid carffgain\r\n");
        } else {
            CarMotionRx_SetGain(float_value);
            Serial_Printf("OK carffgain=%.3f\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "carfflimit", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U) {
            Serial_SendString("ERR invalid carfflimit\r\n");
        } else {
            CarMotionRx_SetAngleLimit(float_value);
            Serial_Printf("OK carfflimit=%.3f deg\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "carffdead", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U) {
            Serial_SendString("ERR invalid carffdead\r\n");
        } else {
            CarMotionRx_SetDeadband(float_value);
            Serial_Printf("OK carffdead=%.4f m/s2\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "carffdir", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            (float_value != 1.0f && float_value != -1.0f)) {
            Serial_SendString("ERR carffdir must be 1 or -1\r\n");
        } else {
            CarMotionRx_SetDirection((float_value < 0.0f) ? -1 : 1);
            Serial_Printf("OK carffdir=%d\r\n", (float_value < 0.0f) ? -1 : 1);
        }
    } else if (Vofa_ParseAssignment(command, "imugain", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U) {
            Serial_SendString("ERR invalid imugain\r\n");
        } else {
            ImuFeedforward_SetGain(float_value);
            Serial_Printf("OK imugain=%.3f\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "imulimit", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U) {
            Serial_SendString("ERR invalid imulimit\r\n");
        } else {
            ImuFeedforward_SetAngleLimit(float_value);
            Serial_Printf("OK imulimit=%.3f\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "imudead", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U) {
            Serial_SendString("ERR invalid imudead\r\n");
        } else {
            ImuFeedforward_SetDeadband(float_value);
            Serial_Printf("OK imudead=%.3f\r\n", (double)float_value);
        }
    } else if (Vofa_ParseAssignment(command, "imudir", &value_text) != 0U) {
        if (Vofa_ParseFloat(value_text, &float_value) == 0U ||
            (float_value != 1.0f && float_value != -1.0f)) {
            Serial_SendString("ERR imudir must be 1 or -1\r\n");
        } else {
            ImuFeedforward_SetDirection((float_value < 0.0f) ? -1 : 1);
            Serial_Printf("OK imudir=%d\r\n", (float_value < 0.0f) ? -1 : 1);
        }
    } else if (Vofa_ParseAssignment(command, "kp", &value_text) != 0U) {
        if (Vofa_UpdateBalanceFloat("kp", value_text) == 0U) Serial_SendString("ERR invalid kp\r\n");
        else Serial_SendString("OK kp\r\n");
    } else if (Vofa_ParseAssignment(command, "ki", &value_text) != 0U) {
        if (Vofa_UpdateBalanceFloat("ki", value_text) == 0U) Serial_SendString("ERR invalid ki\r\n");
        else Serial_SendString("OK ki\r\n");
    } else if (Vofa_ParseAssignment(command, "kd", &value_text) != 0U) {
        if (Vofa_UpdateBalanceFloat("kd", value_text) == 0U) Serial_SendString("ERR invalid kd\r\n");
        else Serial_SendString("OK kd\r\n");
    } else if (Vofa_ParseAssignment(command, "arm", &value_text) != 0U) {
        if (Vofa_UpdateBalanceFloat("arm", value_text) == 0U) Serial_SendString("ERR invalid arm\r\n");
        else Serial_SendString("OK arm\r\n");
    } else if (Vofa_ParseAssignment(command, "maxangle", &value_text) != 0U) {
        if (Vofa_UpdateBalanceFloat("maxangle", value_text) == 0U) Serial_SendString("ERR invalid maxangle\r\n");
        else Serial_SendString("OK maxangle\r\n");
    } else if (Vofa_ParseAssignment(command, "travel", &value_text) != 0U) {
        if (Vofa_UpdateBalanceFloat("travel", value_text) == 0U) Serial_SendString("ERR invalid travel\r\n");
        else Serial_SendString("OK travel\r\n");
    } else {
        Serial_SendString("ERR unknown command; send help\r\n");
    }
}

void VofaTuning_Init(uint32_t now_ms)
{
    s_command_length = 0U;
    s_command_overflow = 0U;
    s_plot_mode = VOFA_PLOT_OFF;
    s_plot_period_ms = VOFA_PLOT_DEFAULT_MS;
    s_last_plot_ms = now_ms;
    s_test_state = TEST3_IDLE;
    s_test_start_ms = now_ms;
    s_test_stage_ms = now_ms;
    s_test_positive_confirm_ms = now_ms;
    s_test_positive_confirm_active = 0U;
    s_test_positive_reached = 0U;
    s_test_positive_turn_cm = 0.0f;
    s_test_positive_error_cm = 100.0f;
    s_motor_test_state = MOTOR_TEST_IDLE;
    s_motor_test_start_ms = now_ms;
    s_motor_test_wait_ms = now_ms;
    s_level_move_state = LEVEL_MOVE_IDLE;
    s_level_move_settle_ms = now_ms;
    s_level_default_raw_deg = BALANCE_LEVEL_DEFAULT_RAW_DEG;
    s_level_default_valid = BALANCE_LEVEL_DEFAULT_VALID_ON_BOOT;
    s_remote_pending_mission = CAR_MOTION_MISSION_NONE;
    s_remote_active_mission = CAR_MOTION_MISSION_NONE;
    s_remote_target_latched = 0U;
    s_remote_captured_target_cm = 0.0f;
    s_remote_last_retry_ms = now_ms;

    Serial_SendString(
        "\r\n========================================\r\n"
        " H2026 balance board - clean VOFA build\r\n"
        " UART0 PB0/PB1 115200; camera UART2 PB15/PB16\r\n"
        " car UART1 PB7(RX)/PB6(TX) 115200 full-duplex task handshake\r\n"
        " PB8=req3 start/stop; PB5=save current track 0deg; buttons active-low to GND\r\n"
        " car PB11=req45 center; car PB10=req6 capture current ball target\r\n"
        " MPU6050 I2C1 PA16/PA29 retained\r\n"
        " Send help followed by Enter\r\n"
        "========================================\r\n");
}

void VofaTuning_Process(uint32_t now_ms)
{
    uint8_t byte;
    uint32_t received = 0U;

    while (received < VOFA_RX_BYTES_PER_CALL && Serial_TryReadByte(&byte)) {
        received++;

        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            if (s_command_overflow != 0U) {
                Serial_SendString("ERR command too long\r\n");
            } else if (s_command_length > 0U) {
                s_command_buffer[s_command_length] = '\0';
                Vofa_ProcessCommand(s_command_buffer, now_ms);
            }
            s_command_length = 0U;
            s_command_overflow = 0U;
            continue;
        }

        if (s_command_overflow != 0U) {
            continue;
        }

        if (s_command_length + 1U >= VOFA_COMMAND_BUFFER_SIZE) {
            s_command_overflow = 1U;
        } else {
            s_command_buffer[s_command_length++] = (char)byte;
        }
    }


    /* Process car PB11/PB10 start requests before generating the ACK frame. */
    Vofa_ProcessCarMission(now_ms);

    /* PB21 remains spare. PB4 restores the configured default level. */
    if (Key_Check(KEY_1, KEY_DOWN) != 0U) {
        Serial_SendString("KEY PB21 reserved; no mission assigned\r\n");
    }
    if (Key_Check(KEY_2, KEY_DOWN) != 0U) {
        Vofa_StartLevelMove(now_ms);
        if (s_level_move_state != LEVEL_MOVE_IDLE) {
            Serial_SendString(
                "KEY PB4: returning to default level; horizontal zero will be set after arrival\r\n");
        }
    }

    /* PB5: stop safely and redefine the current mechanism as track 0 degrees. */
    if (Key_Check(KEY_3, KEY_DOWN) != 0U) {
        Vofa_SaveLevelHere();
        Serial_SendString(
            "KEY PB5: current balance track angle saved as 0 degrees\r\n");
    }

    /*
     * PB8: requirement-3 start button. A second press while any controller or
     * test is active is treated as an emergency stop; the next press starts
     * requirement 3 again.
     */
    if (Key_Check(KEY_4, KEY_DOWN) != 0U) {
        if ((s_test_state != TEST3_IDLE &&
             s_test_state != TEST3_FINISHED) ||
            BallBalance_IsEnabled() != 0U ||
            BallBalance_IsRequirement4Mode() != 0U ||
            s_motor_test_state != MOTOR_TEST_IDLE ||
            s_level_move_state != LEVEL_MOVE_IDLE) {
            BallBalance_Stop();
            CL_StopAll();
            s_test_state = TEST3_IDLE;
            Vofa_ClearRemoteMission();
            Vofa_CancelMotorTest();
            Vofa_CancelLevelMove();
            Serial_SendString("KEY PB8: emergency stop\r\n");
        } else {
            Vofa_StartTest3(now_ms);
            if (s_test_state == TEST3_TO_POSITIVE) {
                Serial_SendString("KEY PB8: requirement 3 started\r\n");
            }
        }
    }

    Vofa_UpdateCarAck(now_ms);

    Vofa_ProcessTest3(now_ms);
    Vofa_ProcessMotorTest(now_ms);
    Vofa_ProcessLevelMove(now_ms);

    if (s_plot_mode != VOFA_PLOT_OFF &&
        (uint32_t)(now_ms - s_last_plot_ms) >= s_plot_period_ms) {
        s_last_plot_ms = now_ms;
        Vofa_PrintPlot();
    }
}
