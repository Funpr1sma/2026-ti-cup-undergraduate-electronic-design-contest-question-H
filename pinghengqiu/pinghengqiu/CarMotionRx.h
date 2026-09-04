#ifndef CAR_MOTION_RX_H_
#define CAR_MOTION_RX_H_

#include <stdint.h>

#define CAR_MOTION_RX_FRAME_SIZE          (9U)
#define CAR_MOTION_RX_HEADER_0             (0xA5U)
#define CAR_MOTION_RX_HEADER_1             (0x5AU)

#define CAR_MOTION_FLAG_RUNNING            (1U << 0)
#define CAR_MOTION_FLAG_EMERGENCY          (1U << 1)
#define CAR_MOTION_FLAG_READY              (1U << 2)
#define CAR_MOTION_FLAG_BALANCE_MODE       (1U << 3)
#define CAR_MOTION_FLAG_PRETILT            (1U << 4)
#define CAR_MOTION_FLAG_REQUIREMENT6       (1U << 5)

#define CAR_MOTION_MISSION_NONE            (0U)
#define CAR_MOTION_MISSION_CENTER          (45U)
#define CAR_MOTION_MISSION_CAPTURED        (6U)

#define BALANCE_ACK_LEVEL_READY            (1U << 0)
#define BALANCE_ACK_CAMERA_VALID           (1U << 1)
#define BALANCE_ACK_CONTROL_ACTIVE         (1U << 2)
#define BALANCE_ACK_TARGET_LATCHED         (1U << 3)
#define BALANCE_ACK_FAULT                  (1U << 4)
#define BALANCE_ACK_PRETILT_READY          (1U << 5)

typedef struct {
    uint8_t sequence;
    uint8_t flags;
    int16_t planned_cps;
    int16_t acceleration_mm_s2;

    float filtered_acceleration_mps2;
    float feedforward_angle_deg;
    float gain;
    float angle_limit_deg;
    float deadband_mps2;
    int8_t direction_sign;

    uint32_t last_frame_ms;
    uint32_t frame_age_ms;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t duplicate_frame_count;
    uint32_t rx_byte_count;
    uint32_t irq_count;
    uint32_t poll_drain_count;

    uint32_t ack_sent_frame_count;
    uint32_t ack_dropped_frame_count;
    uint32_t ack_sent_byte_count;
    uint8_t ack_tx_busy;
    uint8_t active_mission;
    uint8_t pending_start_request;

    uint8_t link_valid;
    uint8_t motion_active;
    uint8_t running_active;
    uint8_t pretilt_active;
    uint8_t emergency_active;
} CarMotionRxSnapshot_t;

void CarMotionRx_Init(void);
void CarMotionRx_Poll(void);
void CarMotionRx_Process(uint32_t now_ms);
void CarMotionRx_TxTask(uint32_t now_ms);

void CarMotionRx_SetGain(float gain);
void CarMotionRx_SetAngleLimit(float angle_limit_deg);
void CarMotionRx_SetDeadband(float deadband_mps2);
void CarMotionRx_SetDirection(int8_t sign);

/* Auto-start handshake. A request is generated on the READY rising edge. */
uint8_t CarMotionRx_TakeStartRequest(void);
uint8_t CarMotionRx_GetActiveMission(uint32_t now_ms);
uint8_t CarMotionRx_IsEmergency(uint32_t now_ms);
void CarMotionRx_SetLocalMissionStatus(
    uint8_t mission,
    uint8_t level_ready,
    uint8_t camera_valid,
    uint8_t control_active,
    uint8_t target_latched,
    uint8_t pretilt_ready,
    uint8_t fault_code);

float CarMotionRx_GetFeedforwardAngleDeg(void);
float CarMotionRx_GetAccelerationMps2(void);
uint8_t CarMotionRx_IsLinkValid(uint32_t now_ms);
uint8_t CarMotionRx_IsMotionActive(uint32_t now_ms);
uint8_t CarMotionRx_IsRunningActive(uint32_t now_ms);
uint8_t CarMotionRx_IsPretiltActive(uint32_t now_ms);
void CarMotionRx_GetSnapshot(CarMotionRxSnapshot_t *snapshot);
void CarMotionRx_ResetDiagnostics(void);

#endif /* CAR_MOTION_RX_H_ */
