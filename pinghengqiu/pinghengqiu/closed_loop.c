/**
  ******************************************************************************
  * @file    closed_loop.c
  * @brief   Stepper motor position closed-loop control
  ******************************************************************************
  */

#include "closed_loop.h"
#include "encoder.h"
#include "demo_config.h"

/*
 * CL_Process() must be called every CL_PERIOD_MS, currently 5 ms.
 */

/* Position must remain in tolerance for this many cycles. */
#define CL_SETTLE_CYCLES             3U

/* Ignore feedback briefly after starting or reversing. */
#define CL_FEEDBACK_GRACE_CYCLES     3U

/*
 * At least one encoder count should normally be observed every 5 ms while
 * running. Declare encoder failure after 20 ineffective cycles = 100 ms.
 */
#define CL_MOVE_CONFIRM_COUNTS       1
#define CL_NO_MOVE_LIMIT_CYCLES      20U

/*
 * Declare wrong direction only after several consecutive reverse movements.
 */
#define CL_REVERSE_CONFIRM_COUNTS    2
#define CL_REVERSE_LIMIT_CYCLES      3U

/*
 * Frequency ramp parameters.
 *
 * Starting at 300 Hz reduces startup impact.
 * The large-error bands are raised by about 1 kHz so the slow lead screw can
 * cross the neutral position sooner.  The ramp is increased only slightly to
 * avoid a hard launch or missed steps.
 */
#define CL_START_FREQUENCY_HZ        300U
#define CL_ACCEL_PER_CYCLE_HZ        350U
#define CL_DECEL_PER_CYCLE_HZ        600U

/*
 * Temporary software travel limit.
 *
 * With a 1 mm/rev lead screw:
 *   1800 degrees = 5 revolutions = about 5 mm
 *
 * These limits are relative to the position where CL_SetZero() was called.
 * Adjust only after measuring the actual safe mechanical travel.
 */
#define CL_MIN_TARGET_DEG            (-1800.0f)
#define CL_MAX_TARGET_DEG            ( 1800.0f)

typedef struct {
    volatile int32_t target_count;
    volatile uint8_t active;
    volatile uint8_t reached;
    volatile CL_Fault_t fault;

    uint8_t settle_cycles;
    int8_t expected_sign;
    uint8_t positive_dir_level;

    int32_t feedback_last_pos;
    uint8_t feedback_grace_cycles;
    uint8_t no_move_cycles;
    uint8_t reverse_cycles;

    uint32_t command_frequency;
} CL_State_t;

static CL_State_t s_cl;
static uint8_t s_initialized;

/* Convert motor shaft angle to encoder counts. */
static int32_t CL_AngleToCount(float angle)
{
    float scaled;

    scaled = angle * (float)ENCODER_COUNTS_PER_REV / 360.0f;

    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }

    return (int32_t)(scaled - 0.5f);
}

/* Convert encoder counts to motor shaft angle. */
static float CL_CountToAngle(int32_t count)
{
    return (float)count * 360.0f /
           (float)ENCODER_COUNTS_PER_REV;
}

/*
 * Select target STEP frequency from position error.
 *
 * Encoder: 4000 counts/rev
 * Motor:   1600 steps/rev at 8 microsteps
 */
static uint32_t CL_SelectFrequency(uint32_t error)
{
    if (error > 2400U) {
        return 9000U;
    }
    if (error > 1200U) {
        return 8000U;
    }
    if (error > 600U) {
        return 6500U;
    }
    if (error > 250U) {
        return 4000U;
    }
    if (error > 80U) {
        return 1800U;
    }
    if (error > 20U) {
        return 700U;
    }

    return 300U;
}

/* Apply acceleration and deceleration limits. */
static uint32_t CL_RampFrequency(
    uint32_t current,
    uint32_t target)
{
    if (target < MOTOR_MIN_FREQ_HZ) {
        target = MOTOR_MIN_FREQ_HZ;
    }

    if (target > MOTOR_MAX_FREQ_HZ) {
        target = MOTOR_MAX_FREQ_HZ;
    }

    if (current == 0U) {
        if (target < CL_START_FREQUENCY_HZ) {
            return target;
        }

        return CL_START_FREQUENCY_HZ;
    }

    if (current < target) {
        uint32_t difference;

        difference = target - current;

        if (difference <= CL_ACCEL_PER_CYCLE_HZ) {
            return target;
        }

        return current + CL_ACCEL_PER_CYCLE_HZ;
    }

    if (current > target) {
        uint32_t difference;

        difference = current - target;

        if (difference <= CL_DECEL_PER_CYCLE_HZ) {
            return target;
        }

        return current - CL_DECEL_PER_CYCLE_HZ;
    }

    return current;
}

/* Reset encoder feedback monitoring from the supplied position. */
static void CL_ResetFeedback(int32_t current)
{
    s_cl.feedback_last_pos = current;
    s_cl.feedback_grace_cycles = 0U;
    s_cl.no_move_cycles = 0U;
    s_cl.reverse_cycles = 0U;
}

/* Stop motor and clear the current motion command state. */
static void CL_ResetMotion(int32_t current)
{
    Motor_Stop(MOTOR_AXIS_X);

    s_cl.command_frequency = 0U;
    s_cl.expected_sign = 0;
    CL_ResetFeedback(current);
}

/* Enter a fault state and stop STEP output immediately. */
static void CL_SetFault(CL_Fault_t fault)
{
    int32_t current;

    Motor_Stop(MOTOR_AXIS_X);
    current = Encoder_GetCount(ENCODER_AXIS_X);

    s_cl.fault = fault;
    s_cl.active = 0U;
    s_cl.reached = 0U;
    s_cl.settle_cycles = 0U;
    s_cl.command_frequency = 0U;
    s_cl.expected_sign = 0;

    CL_ResetFeedback(current);
}

/*
 * Check encoder movement once every 5 ms.
 *
 * A fault is not generated from one abnormal sample:
 * - no movement must persist for CL_NO_MOVE_LIMIT_CYCLES;
 * - reverse movement must persist for CL_REVERSE_LIMIT_CYCLES.
 */
static void CL_CheckFeedback(int32_t current)
{
    int32_t movement;
    uint8_t moving_correctly;
    uint8_t moving_in_reverse;

    movement = current - s_cl.feedback_last_pos;
    s_cl.feedback_last_pos = current;

    if (s_cl.expected_sign == 0 ||
        s_cl.command_frequency == 0U) {
        s_cl.no_move_cycles = 0U;
        s_cl.reverse_cycles = 0U;
        return;
    }

    if (s_cl.feedback_grace_cycles > 0U) {
        s_cl.feedback_grace_cycles--;
        return;
    }

    moving_correctly = 0U;
    moving_in_reverse = 0U;

    if (s_cl.expected_sign > 0) {
        if (movement >= CL_MOVE_CONFIRM_COUNTS) {
            moving_correctly = 1U;
        } else if (movement <= -CL_REVERSE_CONFIRM_COUNTS) {
            moving_in_reverse = 1U;
        }
    } else {
        if (movement <= -CL_MOVE_CONFIRM_COUNTS) {
            moving_correctly = 1U;
        } else if (movement >= CL_REVERSE_CONFIRM_COUNTS) {
            moving_in_reverse = 1U;
        }
    }

    if (moving_correctly != 0U) {
        s_cl.no_move_cycles = 0U;
        s_cl.reverse_cycles = 0U;
        return;
    }

    if (moving_in_reverse != 0U) {
        s_cl.no_move_cycles = 0U;

        if (s_cl.reverse_cycles < 255U) {
            s_cl.reverse_cycles++;
        }

        if (s_cl.reverse_cycles >= CL_REVERSE_LIMIT_CYCLES) {
            CL_SetFault(CL_FAULT_DIRECTION);
        }

        return;
    }

    /*
     * Small encoder jitter is treated as no effective movement.
     * It also breaks the consecutive reverse-movement sequence.
     */
    s_cl.reverse_cycles = 0U;

    if (s_cl.no_move_cycles < 255U) {
        s_cl.no_move_cycles++;
    }

    if (s_cl.no_move_cycles >= CL_NO_MOVE_LIMIT_CYCLES) {
        CL_SetFault(CL_FAULT_NO_ENCODER);
    }
}

void CL_Init(void)
{
    s_initialized = 0U;

    Motor_Stop(MOTOR_AXIS_X);
    Encoder_SetZero(ENCODER_AXIS_X);

    s_cl.target_count = 0;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;

    s_cl.settle_cycles = 0U;
    s_cl.expected_sign = 0;
    s_cl.positive_dir_level = AXIS_X_POSITIVE_DIR_LEVEL;

    s_cl.command_frequency = 0U;
    s_cl.feedback_last_pos = 0;
    s_cl.feedback_grace_cycles = 0U;
    s_cl.no_move_cycles = 0U;
    s_cl.reverse_cycles = 0U;

    s_initialized = 1U;
}

/*
 * Call once every 5 ms, after Encoder_Tick(CL_PERIOD_MS).
 */
void CL_Process(void)
{
    int32_t current;
    int32_t error;
    uint32_t error_abs;
    uint32_t target_frequency;
    uint32_t next_frequency;
    uint8_t direction;
    int8_t sign;
    MotorStatus_t motor_status;

    if (s_initialized == 0U) {
        return;
    }

    current = Encoder_GetCount(ENCODER_AXIS_X);

    CL_CheckFeedback(current);

    if (s_cl.fault != CL_FAULT_NONE ||
        s_cl.active == 0U) {
        return;
    }

    error = s_cl.target_count - current;

    if (error >= 0) {
        error_abs = (uint32_t)error;
    } else {
        error_abs = (uint32_t)(-error);
    }

    /*
     * Stop STEP output inside the target tolerance.
     * Keep the loop active so it can correct later position drift.
     */
    if (error_abs <= CL_TOLERANCE_COUNTS) {
        if (s_cl.command_frequency != 0U ||
            Motor_IsBusy(MOTOR_AXIS_X) != 0U) {
            CL_ResetMotion(current);
        }

        if (s_cl.settle_cycles < CL_SETTLE_CYCLES) {
            s_cl.settle_cycles++;
        }

        if (s_cl.settle_cycles >= CL_SETTLE_CYCLES) {
            s_cl.reached = 1U;
        }

        return;
    }

    s_cl.settle_cycles = 0U;
    s_cl.reached = 0U;

    if (error > 0) {
        sign = 1;
        direction = s_cl.positive_dir_level;
    } else {
        sign = -1;
        direction = (uint8_t)!s_cl.positive_dir_level;
    }

    /*
     * Stop before changing DIR, then restart from the low starting frequency.
     */
    if (s_cl.expected_sign != sign) {
        Motor_Stop(MOTOR_AXIS_X);

        s_cl.command_frequency = 0U;
        s_cl.expected_sign = sign;
        s_cl.feedback_last_pos = current;
        s_cl.feedback_grace_cycles =
            CL_FEEDBACK_GRACE_CYCLES;
        s_cl.no_move_cycles = 0U;
        s_cl.reverse_cycles = 0U;
    }

    target_frequency = CL_SelectFrequency(error_abs);

    next_frequency = CL_RampFrequency(
        s_cl.command_frequency,
        target_frequency);

    motor_status = Motor_RunContinuous(
        MOTOR_AXIS_X,
        next_frequency,
        direction);

    if (motor_status != MOTOR_OK) {
        CL_SetFault(CL_FAULT_DRIVER);
        return;
    }

    s_cl.command_frequency = next_frequency;
}

MotorStatus_t CL_SetTargetAngle(
    MotorAxis_t axis,
    float target_deg)
{
    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U ||
        target_deg != target_deg ||
        target_deg < CL_MIN_TARGET_DEG ||
        target_deg > CL_MAX_TARGET_DEG ||
        s_cl.fault != CL_FAULT_NONE) {
        return MOTOR_ERROR;
    }

    s_cl.target_count = CL_AngleToCount(target_deg);
    s_cl.active = 1U;
    s_cl.reached = 0U;
    s_cl.settle_cycles = 0U;

    return MOTOR_OK;
}

void CL_SetZero(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U) {
        return;
    }

    Motor_Stop(axis);
    Encoder_SetZero(ENCODER_AXIS_X);

    s_cl.target_count = 0;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.settle_cycles = 0U;

    s_cl.command_frequency = 0U;
    s_cl.expected_sign = 0;
    CL_ResetFeedback(0);
}

void CL_SetZeroAll(void)
{
    CL_SetZero(MOTOR_AXIS_X);
}

void CL_Stop(MotorAxis_t axis)
{
    int32_t current;

    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U) {
        return;
    }

    Motor_Stop(axis);
    current = Encoder_GetCount(ENCODER_AXIS_X);

    s_cl.target_count = current;
    s_cl.active = 0U;
    s_cl.reached = 0U;
    s_cl.settle_cycles = 0U;

    s_cl.command_frequency = 0U;
    s_cl.expected_sign = 0;
    CL_ResetFeedback(current);
}

void CL_StopAll(void)
{
    CL_Stop(MOTOR_AXIS_X);
}

void CL_ClearFault(MotorAxis_t axis)
{
    int32_t current;

    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U) {
        return;
    }

    Motor_Stop(axis);
    current = Encoder_GetCount(ENCODER_AXIS_X);

    s_cl.target_count = current;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.settle_cycles = 0U;

    s_cl.command_frequency = 0U;
    s_cl.expected_sign = 0;
    CL_ResetFeedback(current);
}

MotorStatus_t CL_TogglePositiveDirLevel(MotorAxis_t axis)
{
    int32_t current;

    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U ||
        Motor_IsBusy(axis) != 0U) {
        return MOTOR_ERROR;
    }

    s_cl.positive_dir_level =
        (uint8_t)!s_cl.positive_dir_level;

    current = Encoder_GetCount(ENCODER_AXIS_X);
    s_cl.expected_sign = 0;
    s_cl.command_frequency = 0U;
    CL_ResetFeedback(current);

    return MOTOR_OK;
}

uint8_t CL_IsReached(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U) {
        return 0U;
    }

    return s_cl.reached;
}

CL_Fault_t CL_GetFault(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U) {
        return CL_FAULT_DRIVER;
    }

    return s_cl.fault;
}

float CL_GetCurrentAngle(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X ||
        s_initialized == 0U) {
        return 0.0f;
    }

    return Encoder_GetAngle(ENCODER_AXIS_X);
}

void CL_GetSnapshot(
    MotorAxis_t axis,
    CL_Snapshot_t *snapshot)
{
    if (axis != MOTOR_AXIS_X ||
        snapshot == 0 ||
        s_initialized == 0U) {
        return;
    }

    snapshot->current_count =
        Encoder_GetCount(ENCODER_AXIS_X);

    snapshot->target_count =
        s_cl.target_count;

    snapshot->error_count =
        snapshot->target_count -
        snapshot->current_count;

    snapshot->current_angle_deg =
        CL_CountToAngle(snapshot->current_count);

    snapshot->target_angle_deg =
        CL_CountToAngle(snapshot->target_count);

    snapshot->active = s_cl.active;
    snapshot->reached = s_cl.reached;
    snapshot->fault = s_cl.fault;
}