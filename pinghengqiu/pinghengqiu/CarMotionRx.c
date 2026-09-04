#include "CarMotionRx.h"

#include <math.h>
#include <string.h>

#include "clock.h"
#include "ti_msp_dl_config.h"

#define CAR_MOTION_RX_DRAIN_LIMIT              (64U)
#define CAR_MOTION_RX_LINK_TIMEOUT_MS          (120U)
#define CAR_MOTION_RX_PROCESS_PERIOD_MS        (20U)
#define CAR_MOTION_RX_GRAVITY_MPS2             (9.80665f)
#define CAR_MOTION_RX_RAD_TO_DEG               (57.2957795f)
#define CAR_MOTION_RX_FILTER_ALPHA             (0.45f)
#define CAR_MOTION_RX_DECAY_ALPHA              (0.60f)

/* Restored launch/runtime acceleration feed-forward calibration. */
#define CAR_MOTION_RX_DEFAULT_GAIN             (0.85f)
#define CAR_MOTION_RX_DEFAULT_LIMIT_DEG        (1.20f)
#define CAR_MOTION_RX_DEFAULT_DEADBAND_MPS2    (0.015f)
#define CAR_MOTION_RX_DEFAULT_DIRECTION        (-1)

#define BALANCE_ACK_FRAME_SIZE                 (7U)
#define BALANCE_ACK_HEADER_0                   (0x5AU)
#define BALANCE_ACK_HEADER_1                   (0xA5U)
#define BALANCE_ACK_TX_PERIOD_MS               (20U)

typedef struct {
    uint8_t buffer[CAR_MOTION_RX_FRAME_SIZE];
    uint8_t index;
    uint8_t last_sequence;
    uint8_t has_sequence;

    volatile uint8_t sequence;
    volatile uint8_t flags;
    volatile int16_t planned_cps;
    volatile int16_t acceleration_mm_s2;
    volatile uint32_t last_frame_ms;
    volatile uint32_t frame_generation;
    volatile uint8_t pending_start_request;

    uint32_t last_process_ms;
    float filtered_acceleration_mps2;
    float feedforward_angle_deg;
    float gain;
    float angle_limit_deg;
    float deadband_mps2;
    int8_t direction_sign;

    uint8_t local_mission;
    uint8_t local_ack_status;
    uint8_t local_fault_code;
    uint8_t ack_sequence;
    uint8_t ack_frame[BALANCE_ACK_FRAME_SIZE];
    uint8_t ack_tx_index;
    uint8_t ack_tx_busy;
    uint32_t ack_last_send_ms;
    uint32_t ack_sent_frame_count;
    uint32_t ack_dropped_frame_count;
    uint32_t ack_sent_byte_count;

    volatile uint32_t valid_frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint32_t duplicate_frame_count;
    volatile uint32_t rx_byte_count;
    volatile uint32_t irq_count;
    volatile uint32_t poll_drain_count;
} CarMotionRxContext_t;

static CarMotionRxContext_t s_rx;

static float CarMotionRx_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float CarMotionRx_Limit(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t CarMotionRx_ReadI16(const uint8_t *data)
{
    uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)raw;
}

static uint8_t Checksum8(const uint8_t *frame, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t index;

    for (index = 0U; index < length; index++) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum;
}

static uint8_t MissionFromFlags(uint8_t flags)
{
    if ((flags & CAR_MOTION_FLAG_BALANCE_MODE) == 0U) {
        return CAR_MOTION_MISSION_NONE;
    }
    return ((flags & CAR_MOTION_FLAG_REQUIREMENT6) != 0U) ?
        CAR_MOTION_MISSION_CAPTURED : CAR_MOTION_MISSION_CENTER;
}

static void CarMotionRx_AcceptFrame(const uint8_t *frame)
{
    uint8_t sequence = frame[2];
    uint8_t old_flags = s_rx.flags;
    uint8_t new_flags = frame[3];
    uint8_t old_mission = MissionFromFlags(old_flags);
    uint8_t new_mission = MissionFromFlags(new_flags);
    uint8_t old_ready = ((old_flags & CAR_MOTION_FLAG_READY) != 0U) ? 1U : 0U;
    uint8_t new_ready = ((new_flags & CAR_MOTION_FLAG_READY) != 0U) ? 1U : 0U;

    if (s_rx.has_sequence != 0U && sequence == s_rx.last_sequence) {
        s_rx.duplicate_frame_count++;
        return;
    }

    s_rx.has_sequence = 1U;
    s_rx.last_sequence = sequence;
    s_rx.sequence = sequence;
    s_rx.flags = new_flags;
    s_rx.planned_cps = CarMotionRx_ReadI16(&frame[4]);
    s_rx.acceleration_mm_s2 = CarMotionRx_ReadI16(&frame[6]);
    s_rx.last_frame_ms = (uint32_t)tick_ms;
    s_rx.frame_generation++;
    s_rx.valid_frame_count++;

    if (new_ready != 0U && new_mission != CAR_MOTION_MISSION_NONE &&
        (old_ready == 0U || old_mission != new_mission)) {
        s_rx.pending_start_request = new_mission;
    }
}

static void CarMotionRx_ProcessByte(uint8_t byte)
{
    s_rx.rx_byte_count++;

    if (s_rx.index == 0U) {
        if (byte == CAR_MOTION_RX_HEADER_0) {
            s_rx.buffer[0] = byte;
            s_rx.index = 1U;
        }
        return;
    }

    if (s_rx.index == 1U) {
        if (byte == CAR_MOTION_RX_HEADER_1) {
            s_rx.buffer[1] = byte;
            s_rx.index = 2U;
        } else if (byte == CAR_MOTION_RX_HEADER_0) {
            s_rx.buffer[0] = byte;
            s_rx.index = 1U;
        } else {
            s_rx.index = 0U;
        }
        return;
    }

    s_rx.buffer[s_rx.index++] = byte;
    if (s_rx.index < CAR_MOTION_RX_FRAME_SIZE) {
        return;
    }

    s_rx.index = 0U;
    if (Checksum8(s_rx.buffer, CAR_MOTION_RX_FRAME_SIZE - 1U) !=
        s_rx.buffer[CAR_MOTION_RX_FRAME_SIZE - 1U]) {
        s_rx.checksum_error_count++;
        return;
    }

    CarMotionRx_AcceptFrame(s_rx.buffer);
}

static uint32_t CarMotionRx_Drain(uint32_t limit)
{
    uint32_t drained = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST) && drained < limit) {
        CarMotionRx_ProcessByte(DL_UART_Main_receiveData(UART_1_INST));
        drained++;
    }
    return drained;
}

static void AckServiceTx(void)
{
    while (s_rx.ack_tx_busy != 0U &&
           s_rx.ack_tx_index < BALANCE_ACK_FRAME_SIZE) {
        if (!DL_UART_Main_transmitDataCheck(
                UART_1_INST,
                s_rx.ack_frame[s_rx.ack_tx_index])) {
            return;
        }
        s_rx.ack_tx_index++;
        s_rx.ack_sent_byte_count++;
    }

    if (s_rx.ack_tx_busy != 0U &&
        s_rx.ack_tx_index >= BALANCE_ACK_FRAME_SIZE) {
        s_rx.ack_tx_busy = 0U;
        s_rx.ack_tx_index = 0U;
        s_rx.ack_sent_frame_count++;
    }
}

static void AckBuildFrame(void)
{
    s_rx.ack_frame[0] = BALANCE_ACK_HEADER_0;
    s_rx.ack_frame[1] = BALANCE_ACK_HEADER_1;
    s_rx.ack_frame[2] = s_rx.ack_sequence++;
    s_rx.ack_frame[3] = s_rx.local_ack_status;
    s_rx.ack_frame[4] = s_rx.local_mission;
    s_rx.ack_frame[5] = s_rx.local_fault_code;
    s_rx.ack_frame[6] = Checksum8(s_rx.ack_frame, 6U);
    s_rx.ack_tx_index = 0U;
    s_rx.ack_tx_busy = 1U;
}

void CarMotionRx_Init(void)
{
    memset(&s_rx, 0, sizeof(s_rx));
    s_rx.gain = CAR_MOTION_RX_DEFAULT_GAIN;
    s_rx.angle_limit_deg = CAR_MOTION_RX_DEFAULT_LIMIT_DEG;
    s_rx.deadband_mps2 = CAR_MOTION_RX_DEFAULT_DEADBAND_MPS2;
    s_rx.direction_sign = CAR_MOTION_RX_DEFAULT_DIRECTION;
    s_rx.last_process_ms = (uint32_t)tick_ms;
    s_rx.ack_last_send_ms = (uint32_t)tick_ms;

    (void)CarMotionRx_Drain(CAR_MOTION_RX_DRAIN_LIMIT);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void CarMotionRx_Poll(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t drained;

    __disable_irq();
    drained = CarMotionRx_Drain(CAR_MOTION_RX_DRAIN_LIMIT);
    if (drained > 0U) {
        s_rx.poll_drain_count++;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t CarMotionRx_IsLinkValid(uint32_t now_ms)
{
    uint32_t last_frame_ms;
    uint32_t frame_count;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    last_frame_ms = s_rx.last_frame_ms;
    frame_count = s_rx.valid_frame_count;
    if (primask == 0U) {
        __enable_irq();
    }

    return (frame_count != 0U) &&
        ((uint32_t)(now_ms - last_frame_ms) <=
         CAR_MOTION_RX_LINK_TIMEOUT_MS);
}

static uint8_t CarMotionRx_GetFlagsIfValid(uint32_t now_ms)
{
    uint8_t flags;
    uint32_t primask;

    if (CarMotionRx_IsLinkValid(now_ms) == 0U) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    flags = s_rx.flags;
    if (primask == 0U) {
        __enable_irq();
    }
    return flags;
}

uint8_t CarMotionRx_GetActiveMission(uint32_t now_ms)
{
    return MissionFromFlags(CarMotionRx_GetFlagsIfValid(now_ms));
}

uint8_t CarMotionRx_IsEmergency(uint32_t now_ms)
{
    return ((CarMotionRx_GetFlagsIfValid(now_ms) &
             CAR_MOTION_FLAG_EMERGENCY) != 0U) ? 1U : 0U;
}

uint8_t CarMotionRx_IsRunningActive(uint32_t now_ms)
{
    uint8_t flags = CarMotionRx_GetFlagsIfValid(now_ms);

    return ((flags & CAR_MOTION_FLAG_RUNNING) != 0U) &&
           ((flags & CAR_MOTION_FLAG_BALANCE_MODE) != 0U) &&
           ((flags & CAR_MOTION_FLAG_EMERGENCY) == 0U);
}

uint8_t CarMotionRx_IsPretiltActive(uint32_t now_ms)
{
    uint8_t flags = CarMotionRx_GetFlagsIfValid(now_ms);

    return ((flags & CAR_MOTION_FLAG_PRETILT) != 0U) &&
           ((flags & CAR_MOTION_FLAG_READY) != 0U) &&
           ((flags & CAR_MOTION_FLAG_BALANCE_MODE) != 0U) &&
           ((flags & CAR_MOTION_FLAG_EMERGENCY) == 0U);
}

uint8_t CarMotionRx_IsMotionActive(uint32_t now_ms)
{
    /*
     * Before launch the car sends READY + PRETILT together with the planned
     * launch acceleration.  Once RUNNING begins, the same filter transitions
     * directly to the measured motion-planner acceleration without dropping
     * the feed-forward angle to zero.
     */
    return (CarMotionRx_IsRunningActive(now_ms) != 0U ||
            CarMotionRx_IsPretiltActive(now_ms) != 0U) ? 1U : 0U;
}

void CarMotionRx_Process(uint32_t now_ms)
{
    int16_t acceleration_mm_s2;
    uint8_t active;
    float target_acceleration_mps2;
    float effective_acceleration_mps2;
    float target_angle_deg;
    uint32_t primask;

    if ((uint32_t)(now_ms - s_rx.last_process_ms) <
        CAR_MOTION_RX_PROCESS_PERIOD_MS) {
        return;
    }
    s_rx.last_process_ms = now_ms;

    primask = __get_PRIMASK();
    __disable_irq();
    acceleration_mm_s2 = s_rx.acceleration_mm_s2;
    if (primask == 0U) {
        __enable_irq();
    }

    active = CarMotionRx_IsMotionActive(now_ms);
    target_acceleration_mps2 =
        (active != 0U) ? ((float)acceleration_mm_s2 * 0.001f) : 0.0f;

    if (CarMotionRx_IsLinkValid(now_ms) != 0U) {
        s_rx.filtered_acceleration_mps2 +=
            CAR_MOTION_RX_FILTER_ALPHA *
            (target_acceleration_mps2 - s_rx.filtered_acceleration_mps2);
    } else {
        s_rx.filtered_acceleration_mps2 +=
            CAR_MOTION_RX_DECAY_ALPHA *
            (0.0f - s_rx.filtered_acceleration_mps2);
    }

    effective_acceleration_mps2 = s_rx.filtered_acceleration_mps2;
    if (CarMotionRx_Abs(effective_acceleration_mps2) <
        s_rx.deadband_mps2) {
        effective_acceleration_mps2 = 0.0f;
    } else if (effective_acceleration_mps2 > 0.0f) {
        effective_acceleration_mps2 -= s_rx.deadband_mps2;
    } else {
        effective_acceleration_mps2 += s_rx.deadband_mps2;
    }

    target_angle_deg =
        (float)s_rx.direction_sign * s_rx.gain *
        atanf(effective_acceleration_mps2 /
              CAR_MOTION_RX_GRAVITY_MPS2) *
        CAR_MOTION_RX_RAD_TO_DEG;

    s_rx.feedforward_angle_deg = CarMotionRx_Limit(
        target_angle_deg,
        -s_rx.angle_limit_deg,
        s_rx.angle_limit_deg);
}

void CarMotionRx_TxTask(uint32_t now_ms)
{
    AckServiceTx();

    if ((uint32_t)(now_ms - s_rx.ack_last_send_ms) <
        BALANCE_ACK_TX_PERIOD_MS) {
        return;
    }
    s_rx.ack_last_send_ms += BALANCE_ACK_TX_PERIOD_MS;
    if ((uint32_t)(now_ms - s_rx.ack_last_send_ms) >=
        BALANCE_ACK_TX_PERIOD_MS) {
        s_rx.ack_last_send_ms = now_ms;
    }

    if (s_rx.ack_tx_busy != 0U) {
        s_rx.ack_dropped_frame_count++;
        return;
    }

    AckBuildFrame();
    AckServiceTx();
}

uint8_t CarMotionRx_TakeStartRequest(void)
{
    uint8_t request;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    request = s_rx.pending_start_request;
    s_rx.pending_start_request = CAR_MOTION_MISSION_NONE;
    if (primask == 0U) {
        __enable_irq();
    }
    return request;
}

void CarMotionRx_SetLocalMissionStatus(
    uint8_t mission,
    uint8_t level_ready,
    uint8_t camera_valid,
    uint8_t control_active,
    uint8_t target_latched,
    uint8_t pretilt_ready,
    uint8_t fault_code)
{
    uint8_t status = 0U;

    if (level_ready != 0U) {
        status |= BALANCE_ACK_LEVEL_READY;
    }
    if (camera_valid != 0U) {
        status |= BALANCE_ACK_CAMERA_VALID;
    }
    if (control_active != 0U) {
        status |= BALANCE_ACK_CONTROL_ACTIVE;
    }
    if (target_latched != 0U) {
        status |= BALANCE_ACK_TARGET_LATCHED;
    }
    if (pretilt_ready != 0U) {
        status |= BALANCE_ACK_PRETILT_READY;
    }
    if (fault_code != 0U) {
        status |= BALANCE_ACK_FAULT;
    }

    s_rx.local_mission = mission;
    s_rx.local_ack_status = status;
    s_rx.local_fault_code = fault_code;
}

void CarMotionRx_SetGain(float gain)
{
    if (gain == gain) {
        s_rx.gain = CarMotionRx_Limit(gain, 0.0f, 2.0f);
    }
}

void CarMotionRx_SetAngleLimit(float angle_limit_deg)
{
    if (angle_limit_deg == angle_limit_deg) {
        s_rx.angle_limit_deg = CarMotionRx_Limit(
            CarMotionRx_Abs(angle_limit_deg), 0.0f, 3.0f);
    }
}

void CarMotionRx_SetDeadband(float deadband_mps2)
{
    if (deadband_mps2 == deadband_mps2) {
        s_rx.deadband_mps2 = CarMotionRx_Limit(
            CarMotionRx_Abs(deadband_mps2), 0.0f, 1.0f);
    }
}

void CarMotionRx_SetDirection(int8_t sign)
{
    s_rx.direction_sign = (sign < 0) ? -1 : 1;
}

float CarMotionRx_GetFeedforwardAngleDeg(void)
{
    return s_rx.feedforward_angle_deg;
}

float CarMotionRx_GetAccelerationMps2(void)
{
    return s_rx.filtered_acceleration_mps2;
}

void CarMotionRx_GetSnapshot(CarMotionRxSnapshot_t *snapshot)
{
    uint32_t now_ms;
    uint32_t primask;

    if (snapshot == 0) {
        return;
    }

    now_ms = (uint32_t)tick_ms;
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->sequence = s_rx.sequence;
    snapshot->flags = s_rx.flags;
    snapshot->planned_cps = s_rx.planned_cps;
    snapshot->acceleration_mm_s2 = s_rx.acceleration_mm_s2;
    snapshot->last_frame_ms = s_rx.last_frame_ms;
    snapshot->valid_frame_count = s_rx.valid_frame_count;
    snapshot->checksum_error_count = s_rx.checksum_error_count;
    snapshot->duplicate_frame_count = s_rx.duplicate_frame_count;
    snapshot->rx_byte_count = s_rx.rx_byte_count;
    snapshot->irq_count = s_rx.irq_count;
    snapshot->poll_drain_count = s_rx.poll_drain_count;
    snapshot->pending_start_request = s_rx.pending_start_request;
    if (primask == 0U) {
        __enable_irq();
    }

    snapshot->filtered_acceleration_mps2 = s_rx.filtered_acceleration_mps2;
    snapshot->feedforward_angle_deg = s_rx.feedforward_angle_deg;
    snapshot->gain = s_rx.gain;
    snapshot->angle_limit_deg = s_rx.angle_limit_deg;
    snapshot->deadband_mps2 = s_rx.deadband_mps2;
    snapshot->direction_sign = s_rx.direction_sign;
    snapshot->frame_age_ms =
        (snapshot->valid_frame_count == 0U) ? 0xFFFFFFFFU :
        (uint32_t)(now_ms - snapshot->last_frame_ms);
    snapshot->link_valid = CarMotionRx_IsLinkValid(now_ms);
    snapshot->motion_active = CarMotionRx_IsMotionActive(now_ms);
    snapshot->running_active = CarMotionRx_IsRunningActive(now_ms);
    snapshot->pretilt_active = CarMotionRx_IsPretiltActive(now_ms);
    snapshot->emergency_active = CarMotionRx_IsEmergency(now_ms);
    snapshot->active_mission = CarMotionRx_GetActiveMission(now_ms);
    snapshot->ack_sent_frame_count = s_rx.ack_sent_frame_count;
    snapshot->ack_dropped_frame_count = s_rx.ack_dropped_frame_count;
    snapshot->ack_sent_byte_count = s_rx.ack_sent_byte_count;
    snapshot->ack_tx_busy = s_rx.ack_tx_busy;
}

void CarMotionRx_ResetDiagnostics(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_rx.index = 0U;
    s_rx.has_sequence = 0U;
    s_rx.flags = 0U;
    s_rx.frame_generation = 0U;
    s_rx.valid_frame_count = 0U;
    s_rx.checksum_error_count = 0U;
    s_rx.duplicate_frame_count = 0U;
    s_rx.rx_byte_count = 0U;
    s_rx.irq_count = 0U;
    s_rx.poll_drain_count = 0U;
    s_rx.pending_start_request = CAR_MOTION_MISSION_NONE;
    s_rx.ack_sent_frame_count = 0U;
    s_rx.ack_dropped_frame_count = 0U;
    s_rx.ack_sent_byte_count = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

void UART_1_INST_IRQHandler(void)
{
    s_rx.irq_count++;
    (void)DL_UART_Main_getPendingInterrupt(UART_1_INST);
    (void)CarMotionRx_Drain(CAR_MOTION_RX_DRAIN_LIMIT);
}
