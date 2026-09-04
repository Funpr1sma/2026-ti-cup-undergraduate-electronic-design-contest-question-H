#ifndef BALL_BALANCE_CONTROL_H
#define BALL_BALANCE_CONTROL_H

#include <stdint.h>

typedef enum {
    BALL_BALANCE_STATE_NOT_READY = 0,
    BALL_BALANCE_STATE_DISABLED,
    BALL_BALANCE_STATE_RETURNING_ZERO,
    BALL_BALANCE_STATE_RUNNING,
    BALL_BALANCE_STATE_SETTLED,
    BALL_BALANCE_STATE_FAULT
} BallBalanceState_t;

typedef enum {
    BALL_BALANCE_FAULT_NONE = 0,
    BALL_BALANCE_FAULT_CAMERA_TIMEOUT,
    BALL_BALANCE_FAULT_BALL_OUT_OF_RANGE,
    BALL_BALANCE_FAULT_ACTUATOR_OVER_TRAVEL,
    BALL_BALANCE_FAULT_MOTOR_ENCODER,
    BALL_BALANCE_FAULT_MOTOR_DIRECTION,
    BALL_BALANCE_FAULT_MOTOR_DRIVER,
    BALL_BALANCE_FAULT_INVALID_CONFIG
} BallBalanceFault_t;

typedef struct {
    /*
     * 钢球位置 PID。
     *
     * kp: deg/cm
     * ki: deg/(cm*s)
     * kd: deg/(cm/s)
     */
    float kp;
    float ki;
    float kd;

    /* 摆杆控制方向，只允许 +1 或 -1。 */
    int8_t control_sign;

    /* 合页中心到丝杆作用点中心的距离。 */
    float lever_arm_mm;

    /* 电机角度与丝杆位移关系：当前为 180 deg/mm。 */
    float motor_deg_per_mm;

    /* 允许设置的钢球目标范围。 */
    float ball_target_min_cm;
    float ball_target_max_cm;

    /* 检测到钢球超过该范围时立即停止。 */
    float ball_safety_min_cm;
    float ball_safety_max_cm;

    /* 最大摆杆倾角。 */
    float max_flap_angle_deg;

    /* 相对于调平零点的丝杆安全行程。 */
    float actuator_min_mm;
    float actuator_max_mm;

    /* 积分项最大输出角度。 */
    float integral_output_limit_deg;

    /* 钢球速度低通滤波系数，范围 0~1。 */
    float velocity_filter_alpha;

    /* 摄像头数据最大允许间隔。 */
    uint32_t camera_timeout_ms;

    /* 稳定判据。 */
    float settle_position_error_cm;
    float settle_velocity_cm_s;
    uint32_t settle_time_ms;
} BallBalanceConfig_t;

typedef struct {
    float target_ball_cm;
    float current_ball_cm;
    float ball_error_cm;
    float ball_velocity_cm_s;

    float p_output_deg;
    float i_output_deg;
    float d_output_deg;

    /* Requirement-4 telemetry. Requirement 3 leaves feed-forward disabled. */
    float visual_flap_target_deg;
    float feedforward_angle_deg;
    float feedforward_acceleration_mps2;
    float flap_target_deg;

    /* Requirement-4-only visual controller diagnostics. */
    float req4_raw_velocity_cm_s;
    float req4_filtered_velocity_cm_s;
    float req4_predicted_error_cm;
    float req4_breakaway_output_deg;

    float actuator_target_mm;
    float actuator_current_mm;

    float motor_target_deg;
    float motor_current_deg;

    uint32_t measurement_sequence;
    uint32_t measurement_timestamp_ms;

    uint8_t level_calibrated;
    uint8_t enabled;
    uint8_t settled;
    uint8_t feedforward_enabled;
    uint8_t feedforward_valid;
    uint8_t requirement4_mode;
    uint8_t req4_breakaway_active;
    uint8_t req4_visual_saturated;
    uint8_t req4_in_one_cm_band;

    BallBalanceState_t state;
    BallBalanceFault_t fault;
} BallBalanceSnapshot_t;

/*
 * 调用顺序：
 *
 * Motor_Init();
 * Encoder_Init();
 * CL_Init();
 * BallBalance_Init();
 */
void BallBalance_Init(void);

/* 固定每20ms调用一次。 */
void BallBalance_Process20ms(void);

/*
 * 摄像头每得到一次有效钢球位置后调用。
 *
 * position_cm:
 *   中心点为0，右侧为正，左侧为负。
 *
 * timestamp_ms:
 *   建议传入tick_ms。
 */
uint8_t BallBalance_PushBallPosition(
    float position_cm,
    uint32_t timestamp_ms);

uint8_t BallBalance_Enable(uint8_t enable);
uint8_t BallBalance_SetTargetCm(float target_cm);
void BallBalance_Stop(void);

/*
 * 必须先人工将摆杆调平，再调用此函数。
 * 此函数会把当前电机编码器位置设为丝杆0mm。
 */
void BallBalance_SetLevelZero(void);

void BallBalance_ClearFault(void);


/*
 * Requirement-4 external acceleration feed-forward. The feed-forward angle is
 * an actual flap angle and is added after the visual feedback controller.
 * It is disabled by default and never participates in the requirement-3
 * +5 cm -> -5 cm motion unless explicitly enabled.
 */
void BallBalance_SetExternalFeedforwardEnabled(uint8_t enable);
void BallBalance_UpdateExternalFeedforward(
    float angle_deg,
    float acceleration_mps2,
    uint8_t valid);
uint8_t BallBalance_IsExternalFeedforwardEnabled(void);

/* Requirement-4-only visual controller selection. Requirement 3 always
 * forces this mode off and continues using its original tuned controller. */
void BallBalance_SetRequirement4Mode(uint8_t enable);
uint8_t BallBalance_IsRequirement4Mode(void);

uint8_t BallBalance_SetPid(
    float kp,
    float ki,
    float kd);

void BallBalance_GetConfig(
    BallBalanceConfig_t *config);

uint8_t BallBalance_SetConfig(
    const BallBalanceConfig_t *config);

uint8_t BallBalance_IsReady(void);
uint8_t BallBalance_IsEnabled(void);
uint8_t BallBalance_IsSettled(void);

BallBalanceState_t BallBalance_GetState(void);
BallBalanceFault_t BallBalance_GetFault(void);

void BallBalance_GetSnapshot(
    BallBalanceSnapshot_t *snapshot);

#endif