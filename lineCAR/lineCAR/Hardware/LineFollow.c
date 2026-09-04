#include "LineFollow.h"

#include "Config/CarConfig.h"
#include "DebugPrint.h"
#include "GraySensor.h"
#include "Serial.h"
#include "SpeedPI.h"

#include <stdbool.h>
#include <stdint.h>
#include "MotionPlanner.h"

#define LINE_LEFT_WHEEL  (SPEED_PI_MOTOR1)
#define LINE_RIGHT_WHEEL (SPEED_PI_MOTOR2)

static LineFollowStatus_t g_status;
static int32_t g_kpNum = LINE_DEFAULT_KP_NUM;
static int32_t g_kdNum = LINE_DEFAULT_KD_NUM;
static int32_t g_previousError;
static int32_t g_filteredDerivative;
static int32_t g_previousCorrection;
static bool g_errorInitialized;
static uint32_t g_lastControlMs;
static uint32_t g_lastValidMs;
static bool g_timeInitialized;

static int32_t AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t LimitI32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static bool TaskDue(uint32_t nowMs, uint32_t *lastMs, uint32_t periodMs)
{
    if ((uint32_t)(nowMs - *lastMs) < periodMs)
    {
        return false;
    }

    *lastMs += periodMs;
    if ((uint32_t)(nowMs - *lastMs) >= periodMs)
    {
        *lastMs = nowMs;
    }
    return true;
}

static void SetPhysicalWheelTargets(int32_t leftCps, int32_t rightCps)
{
    SpeedPI_SetTarget(LINE_LEFT_WHEEL, leftCps);
    SpeedPI_SetTarget(LINE_RIGHT_WHEEL, rightCps);
}

static void SetLimitedWheelTargets(
    uint32_t nowMs,
    int32_t requestedBaseCps,
    int32_t correctionCps)
{
    int32_t plannedBaseCps;
    int32_t leftTarget;
    int32_t rightTarget;

    MotionPlanner_SetTargetCps(requestedBaseCps);
    plannedBaseCps = MotionPlanner_Update(nowMs);

    leftTarget = LimitI32(
        plannedBaseCps + correctionCps,
        0,
        LINE_MAX_TARGET_CPS);

    rightTarget = LimitI32(
        plannedBaseCps - correctionCps,
        0,
        LINE_MAX_TARGET_CPS);

    g_status.correctionCps = correctionCps;
    g_status.leftTargetCps = leftTarget;
    g_status.rightTargetCps = rightTarget;

    SetPhysicalWheelTargets(leftTarget, rightTarget);
}
static int32_t EffectiveBaseForError(int32_t error)
{
    int32_t absError = AbsI32(error);
    int32_t configuredBase = g_status.baseCps;
    int32_t minimumBase = LINE_SHARP_TURN_MIN_BASE_CPS;
    int32_t errorSpan;
    int32_t errorIntoTurn;
    int32_t reduction;

    if (minimumBase > configuredBase)
    {
        minimumBase = configuredBase;
    }

    if (absError <= LINE_SHARP_TURN_START_ERROR)
    {
        return configuredBase;
    }

    if (absError >= LINE_SHARP_TURN_FULL_ERROR)
    {
        return minimumBase;
    }

    errorSpan = LINE_SHARP_TURN_FULL_ERROR - LINE_SHARP_TURN_START_ERROR;
    errorIntoTurn = absError - LINE_SHARP_TURN_START_ERROR;
    reduction =
        (configuredBase - minimumBase) * errorIntoTurn / errorSpan;

    return configuredBase - reduction;
}

static void ResetDynamicTerms(void)
{
    g_previousError = 0;
    g_filteredDerivative = 0;
    g_previousCorrection = 0;
    g_errorInitialized = false;
    g_status.replayingLastState = false;
    g_status.lineLostElapsedMs = 0U;
    g_status.error = 0;
    g_status.derivative = 0;
    g_status.correctionCps = 0;
}

static void StopForInvalidLine(void)
{
    g_status.position = 0;
    g_status.leftTargetCps = 0;
    g_status.rightTargetCps = 0;
    ResetDynamicTerms();
    MotionPlanner_Reset(g_lastControlMs);
    SpeedPI_StopAll();
}

static int32_t ApplyCorrectionSlew(int32_t requested)
{
    int32_t change = requested - g_previousCorrection;

    change = LimitI32(
        change,
        -LINE_CORRECTION_SLEW_CPS_PER_STEP,
        LINE_CORRECTION_SLEW_CPS_PER_STEP
    );

    g_previousCorrection += change;
    return g_previousCorrection;
}

/*
 * Reuse the last valid steering state through a short all-white gap.
 *
 * The previous correction preserves the current curve direction. The lower
 * replay base speed reduces how far the car can travel before the line returns.
 */
static void ReplayLastValidState(uint32_t nowMs)
{
    int32_t replayBase;
    int32_t replayCorrection;

    replayBase = g_status.baseCps;
    if (replayBase > LINE_LOST_REPLAY_BASE_CPS)
    {
        replayBase = LINE_LOST_REPLAY_BASE_CPS;
    }

    replayCorrection = LimitI32(
        g_previousCorrection,
        -LINE_LOST_REPLAY_MAX_CORRECTION_CPS,
        LINE_LOST_REPLAY_MAX_CORRECTION_CPS
    );

    g_filteredDerivative = 0;
    g_status.replayingLastState = true;
    g_status.lineLostElapsedMs = (uint32_t)(nowMs - g_lastValidMs);
    g_status.position = g_previousError;
    g_status.error = g_previousError;
    g_status.derivative = 0;

    SetLimitedWheelTargets(
    nowMs,
    replayBase,
    replayCorrection);
}

static void UpdateController(uint32_t nowMs)
{
    GraySensorData_t sensor = GraySensor_GetLastData();
    bool wasReplaying = g_status.replayingLastState;
    int32_t rawDerivative;
    int32_t correction;
    int32_t effectiveBase;

    g_status.sensorMask = sensor.mask;
    g_status.lineValid = sensor.lineDetected && sensor.positionValid;

    if (!g_status.enabled)
    {
        StopForInvalidLine();
        return;
    }

    if (!g_status.lineValid)
    {
        uint32_t lostElapsed = (uint32_t)(nowMs - g_lastValidMs);

        g_status.lineLostElapsedMs = lostElapsed;

        if (g_errorInitialized &&
            (lostElapsed <= LINE_LOST_REPLAY_MS))
        {
            ReplayLastValidState(nowMs);
            return;
        }

        StopForInvalidLine();
        return;
    }

    g_lastValidMs = nowMs;
    g_status.replayingLastState = false;
    g_status.lineLostElapsedMs = 0U;
    g_status.position = (int32_t)sensor.position;
    g_status.error = g_status.position;

    /*
     * On reacquisition, reset only the derivative reference. Keep the previous
     * correction so the slew limiter transitions smoothly from replay control.
     */
    if (!g_errorInitialized || wasReplaying)
    {
        g_previousError = g_status.error;
        g_filteredDerivative = 0;
        g_errorInitialized = true;
    }
    else
    {
        rawDerivative =
            (g_status.error - g_previousError) *
            (int32_t)LINE_D_REFERENCE_PERIOD_MS /
            (int32_t)LINE_CONTROL_PERIOD_MS;

        g_previousError = g_status.error;
        rawDerivative = LimitI32(
            rawDerivative,
            -LINE_DERIVATIVE_LIMIT,
            LINE_DERIVATIVE_LIMIT
        );

#if LINE_D_FILTER_SHIFT == 0U
        g_filteredDerivative = rawDerivative;
#else
        g_filteredDerivative +=
            (rawDerivative - g_filteredDerivative) /
            (int32_t)(1U << LINE_D_FILTER_SHIFT);
#endif
    }

    g_status.derivative = g_filteredDerivative;
    correction =
        (g_kpNum * g_status.error +
         g_kdNum * g_status.derivative) /
        LINE_GAIN_DIV;
    correction *= LINE_STEER_SIGN;
    correction = LimitI32(
        correction,
        -LINE_MAX_CORRECTION_CPS,
        LINE_MAX_CORRECTION_CPS
    );
    correction = ApplyCorrectionSlew(correction);

    effectiveBase = EffectiveBaseForError(g_status.error);
    SetLimitedWheelTargets(
    nowMs,
    effectiveBase,
    correction);
}

void LineFollow_Init(void)
{
    g_status = (LineFollowStatus_t){0};
    g_status.baseCps = LINE_DEFAULT_BASE_CPS;

    g_kpNum = LINE_DEFAULT_KP_NUM;
    g_kdNum = LINE_DEFAULT_KD_NUM;
    g_lastControlMs = 0U;
    g_lastValidMs = 0U;
    g_timeInitialized = false;
    ResetDynamicTerms();
    SpeedPI_StopAll();

    Serial_SendString("LineFollow fast-finish: base=");
    DebugPrint_I32(g_status.baseCps, false);
    Serial_SendString(" Kp/Kd=");
    DebugPrint_I32(g_kpNum, false);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_I32(g_kdNum, false);
    Serial_SendString(" period=5ms replay=140ms\r\n");
}

void LineFollow_Enable(void)
{
    g_status.enabled = true;
    ResetDynamicTerms();
}

void LineFollow_Disable(void)
{
    g_status.enabled = false;
    g_status.lineValid = false;
    g_status.sensorMask = 0U;
    StopForInvalidLine();
}

void LineFollow_SetBaseCps(int32_t baseCps)
{
    g_status.baseCps = LimitI32(
        baseCps,
        LINE_MIN_BASE_CPS,
        LINE_MAX_BASE_CPS
    );
}

bool LineFollow_SetGains(int32_t kpNum, int32_t kdNum)
{
    if ((kpNum < LINE_KP_NUM_MIN) || (kpNum > LINE_KP_NUM_MAX) ||
        (kdNum < LINE_KD_NUM_MIN) || (kdNum > LINE_KD_NUM_MAX))
    {
        return false;
    }

    g_kpNum = kpNum;
    g_kdNum = kdNum;
    ResetDynamicTerms();
    return true;
}

void LineFollow_GetGains(int32_t *kpNum, int32_t *kdNum)
{
    if (kpNum != 0)
    {
        *kpNum = g_kpNum;
    }
    if (kdNum != 0)
    {
        *kdNum = g_kdNum;
    }
}

void LineFollow_SetDirectWheelTargets(
    int32_t leftTargetCps,
    int32_t rightTargetCps)
{
    leftTargetCps = LimitI32(
        leftTargetCps,
        -LINE_MAX_TARGET_CPS,
        LINE_MAX_TARGET_CPS
    );
    rightTargetCps = LimitI32(
        rightTargetCps,
        -LINE_MAX_TARGET_CPS,
        LINE_MAX_TARGET_CPS
    );

    g_status.leftTargetCps = leftTargetCps;
    g_status.rightTargetCps = rightTargetCps;
    SetPhysicalWheelTargets(leftTargetCps, rightTargetCps);
}

LineFollowStatus_t LineFollow_GetStatus(void)
{
    return g_status;
}

void LineFollow_PrintStatus(void)
{
    SpeedPIStatus_t speed = SpeedPI_GetStatus();

    Serial_SendString("LINE ");
    Serial_SendString(g_status.enabled ? "ON" : "OFF");
    Serial_SendString(" mask=");
    DebugPrint_Mask8(g_status.sensorMask);
    Serial_SendString(" valid=");
    DebugPrint_Bool(g_status.lineValid);
    Serial_SendString(" replay=");
    DebugPrint_Bool(g_status.replayingLastState);
    Serial_SendString(" lostMs=");
    DebugPrint_U32(g_status.lineLostElapsedMs);
    Serial_SendString(" pos=");
    DebugPrint_I32(g_status.position, true);
    Serial_SendString(" d=");
    DebugPrint_I32(g_status.derivative, true);
    Serial_SendString(" corr=");
    DebugPrint_I32(g_status.correctionCps, true);
    Serial_SendString(" L/R=");
    DebugPrint_I32(g_status.leftTargetCps, true);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_I32(g_status.rightTargetCps, true);
    Serial_SendString(" M1/M2=");
    DebugPrint_I32(speed.motor1.filteredCps, true);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_I32(speed.motor2.filteredCps, true);
    Serial_SendString("\r\n");
}

void LineFollow_ControlTask(uint32_t nowMs)
{
    if (!g_timeInitialized)
    {
        g_lastControlMs = nowMs;
        g_lastValidMs = nowMs;
        g_timeInitialized = true;
    }

    if (TaskDue(nowMs, &g_lastControlMs, LINE_CONTROL_PERIOD_MS))
    {
        UpdateController(nowMs);
    }

    /* Encoder and speed PI retain their independent 20 ms period. */
    SpeedPI_ControlTask(nowMs);
}
