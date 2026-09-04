#include "CarControl.h"

#include "Config/CarConfig.h"
#include "DebugPrint.h"
#include "LineFollow.h"
#include "Serial.h"
#include "SpeedPI.h"
#include "MotionPlanner.h"
#include "CarMotionLink.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool active;
    uint32_t startMs;
} StableTimer_t;

static CarControlStatus_t g_status;

static StableTimer_t g_startMarkerTimer;
static StableTimer_t g_markerLeaveTimer;
static StableTimer_t g_lineLostTimer;
static StableTimer_t g_stopTimer;

static uint32_t g_stateEnterMs;
static uint32_t g_lastSupervisorMs;
static uint32_t g_driveStartMs;
static bool g_timeInitialized;
static bool g_straightTuning;
static bool g_stopToFinished;
static bool g_finishMarkerArmed;
static bool g_balanceMarkerConfirmed;
static uint32_t g_balanceReadyWaitStartMs;

static int32_t AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
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

static void StableTimer_Reset(StableTimer_t *timer)
{
    timer->active = false;
    timer->startMs = 0U;
}

static bool StableTimer_Update(
    StableTimer_t *timer,
    bool condition,
    uint32_t nowMs,
    uint32_t requiredMs)
{
    if (!condition)
    {
        StableTimer_Reset(timer);
        return false;
    }

    if (!timer->active)
    {
        timer->active = true;
        timer->startMs = nowMs;
        return requiredMs == 0U;
    }

    return (uint32_t)(nowMs - timer->startMs) >= requiredMs;
}

static void ResetStableTimers(void)
{
    StableTimer_Reset(&g_startMarkerTimer);
    StableTimer_Reset(&g_markerLeaveTimer);
    StableTimer_Reset(&g_lineLostTimer);
    StableTimer_Reset(&g_stopTimer);
}

static void DriveTimer_Reset(void)
{
    g_driveStartMs = 0U;
    g_status.driveTimeMs = 0U;
    g_status.driveTimerRunning = false;
}

static void DriveTimer_Start(uint32_t nowMs)
{
    g_driveStartMs = nowMs;
    g_status.driveTimeMs = 0U;
    g_status.driveTimerRunning = true;
}

static void DriveTimer_Update(uint32_t nowMs)
{
    if (g_status.driveTimerRunning)
    {
        g_status.driveTimeMs = (uint32_t)(nowMs - g_driveStartMs);
    }
}

static void DriveTimer_Stop(uint32_t nowMs)
{
    DriveTimer_Update(nowMs);
    g_status.driveTimerRunning = false;
}

static bool MissionIsBalance(CarMissionMode_t mode)
{
    return (mode == CAR_MISSION_BALANCE_CENTER) ||
        (mode == CAR_MISSION_BALANCE_CAPTURED_TARGET);
}

static uint8_t MissionLinkCode(CarMissionMode_t mode)
{
    return (mode == CAR_MISSION_BALANCE_CAPTURED_TARGET) ?
        CAR_MOTION_MISSION_CAPTURED : CAR_MOTION_MISSION_CENTER;
}

static void MissionApplyMotionProfile(CarMissionMode_t mode)
{
    if (MissionIsBalance(mode))
    {
        MotionPlanner_SetProfile(
            MOTION_BALANCE_ACCEL_MM_S2,
            MOTION_BALANCE_DECEL_MM_S2);
    }
    else
    {
        MotionPlanner_SetProfile(
            MOTION_REQ2_ACCEL_MM_S2,
            MOTION_REQ2_DECEL_MM_S2);
    }
}

static int32_t MissionDefaultBase(CarMissionMode_t mode)
{
    return MissionIsBalance(mode) ?
        CAR_CONTINUOUS_BASE_CPS : CAR_FAST_BASE_CPS;
}

static const char *MissionName(CarMissionMode_t mode)
{
    switch (mode)
    {
        case CAR_MISSION_BALANCE_CENTER: return "REQ45_CENTER_CONTINUOUS";
        case CAR_MISSION_BALANCE_CAPTURED_TARGET: return "REQ6_CAPTURED_CONTINUOUS";
        case CAR_MISSION_STOP_AT_A: return "STOP_A_2";
        default: return "UNKNOWN";
    }
}

static const char *StateName(CarState_t state)
{
    switch (state)
    {
        case CAR_STATE_IDLE: return "IDLE";
        case CAR_STATE_ARMING: return "ARMING";
        case CAR_STATE_PRETILT: return "PRETILT";
        case CAR_STATE_RUNNING: return "RUNNING";
        case CAR_STATE_PASSING_FINISH: return "PASSING_FINISH";
        case CAR_STATE_STOPPING: return "STOPPING";
        case CAR_STATE_FINISHED: return "FINISHED";
        case CAR_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static const char *FaultName(CarFault_t fault)
{
    switch (fault)
    {
        case CAR_FAULT_NONE: return "NONE";
        case CAR_FAULT_START_NO_LINE: return "START_NO_LINE";
        case CAR_FAULT_LINE_LOST: return "LINE_LOST";
        case CAR_FAULT_BALANCE_NOT_READY: return "BALANCE_NOT_READY";
        default: return "UNKNOWN";
    }
}

static const char *PatternName(GraySensorPattern_t pattern)
{
    switch (pattern)
    {
        case GRAY_PATTERN_LOST: return "LOST";
        case GRAY_PATTERN_NORMAL: return "NORMAL";
        case GRAY_PATTERN_WIDE: return "WIDE";
        case GRAY_PATTERN_SPLIT: return "SPLIT";
        case GRAY_PATTERN_ALL_BLACK: return "ALL";
        default: return "UNKNOWN";
    }
}

static void ReadLine(void)
{
    GraySensorData_t data = GraySensor_ReadData();

    g_status.sensorMask = data.mask;
    g_status.activeCount = data.activeCount;
    g_status.linePattern = data.pattern;
    g_status.lineValid = data.lineDetected && data.positionValid;
    g_status.linePosition = g_status.lineValid ? (int32_t)data.position : 0;

    if (g_status.lineValid && (AbsI32(g_status.linePosition) >= 250))
    {
        g_status.lastLinePosition = g_status.linePosition;
    }
}

static uint8_t MarkerTotalBlackCount(void)
{
    return g_status.activeCount;
}

static bool IsRequiredCenterFiveMarkerMask(uint8_t mask)
{
    /* S3..S7 is intentionally excluded after clockwise-track validation. */
    return mask == CAR_MARKER_CENTER5_S2_TO_S6_MASK;
}

static bool IsMarkerCandidate(void)
{
    return (MarkerTotalBlackCount() >= CAR_MARKER_MIN_TOTAL_BLACK_COUNT) ||
        IsRequiredCenterFiveMarkerMask(g_status.sensorMask);
}

static bool IsStrongMarker(void)
{
    return IsMarkerCandidate();
}

static bool WheelsStopped(void)
{
    SpeedPIStatus_t speed = SpeedPI_GetStatus();

    return (AbsI32(speed.motor1.filteredCps) <= CAR_STOP_CPS) &&
        (AbsI32(speed.motor2.filteredCps) <= CAR_STOP_CPS);
}

static void EnterState(CarState_t newState, uint32_t nowMs)
{
    CarState_t oldState = g_status.state;

    if (oldState == newState)
    {
        return;
    }

    g_status.state = newState;
    g_stateEnterMs = nowMs;
    g_status.stateElapsedMs = 0U;
    ResetStableTimers();

    if (newState != CAR_STATE_ARMING)
    {
        g_balanceMarkerConfirmed = false;
        g_balanceReadyWaitStartMs = 0U;
    }

    if ((newState == CAR_STATE_IDLE) ||
        (newState == CAR_STATE_FINISHED) ||
        (newState == CAR_STATE_FAULT))
    {
        DriveTimer_Stop(nowMs);
    }

    switch (newState)
    {
        case CAR_STATE_IDLE:
            LineFollow_Disable();
            MotionPlanner_Reset(nowMs);
            SpeedPI_SetAllStartupMinimumPercent(0U);
            SpeedPI_StopAll();
            g_straightTuning = false;
            g_stopToFinished = false;
            break;

        case CAR_STATE_ARMING:
            LineFollow_Disable();
            MotionPlanner_Reset(nowMs);
            SpeedPI_SetAllStartupMinimumPercent(0U);
            SpeedPI_StopAll();
            g_balanceMarkerConfirmed = false;
            g_balanceReadyWaitStartMs = 0U;
            break;

        case CAR_STATE_PRETILT:
            /* The car remains actively braked while UART1 sends the intended
             * launch acceleration to the balance board. */
            LineFollow_Disable();
            MotionPlanner_Reset(nowMs);
            SpeedPI_SetAllStartupMinimumPercent(0U);
            SpeedPI_StopAll();
            break;

        case CAR_STATE_RUNNING:
        case CAR_STATE_PASSING_FINISH:
            MotionPlanner_Reset(nowMs);
            SpeedPI_SetAllStartupMinimumPercent(0U);
            if (g_straightTuning)
            {
                LineFollow_Disable();
                SpeedPI_ResetAllIntegrals();
            }
            else
            {
                LineFollow_SetBaseCps(g_status.baseCps);
                LineFollow_Enable();
            }
            break;

        case CAR_STATE_STOPPING:
        case CAR_STATE_FINISHED:
        case CAR_STATE_FAULT:
        default:
            LineFollow_Disable();
            MotionPlanner_Reset(nowMs);
            SpeedPI_SetAllStartupMinimumPercent(0U);
            SpeedPI_StopAll();
            break;
    }

    Serial_SendString("STATE:");
    Serial_SendString(StateName(newState));
    Serial_SendString("\r\n");
}

static void EnterFault(CarFault_t fault, uint32_t nowMs)
{
    g_status.fault = fault;
    EnterState(CAR_STATE_FAULT, nowMs);
    Serial_SendString("FAULT:");
    Serial_SendString(FaultName(fault));
    Serial_SendString("\r\n");
}

static void UpdateArming(uint32_t nowMs)
{
    /*
     * PB11/PB10 missions use a non-blocking two-way handshake:
     *   1. READY + mission type is sent as soon as the button starts ARMING.
     *   2. The balance board starts/latches its controller and returns ACK.
     *   3. After the start marker and matching ACK are present, enter PRETILT.
     *   4. The car remains stopped while the intended launch acceleration is
     *      sent and the balance board moves to the compensating angle.
     *   5. Start the normal motion planner after PRETILT_READY or timeout.
     */
    if (g_balanceMarkerConfirmed)
    {
        uint8_t missionCode = MissionLinkCode(g_status.missionMode);

        if (CarMotionLink_IsBalanceReady(missionCode, nowMs))
        {
            Serial_SendString(
                "BALANCE_ACK ready; enter launch pre-tilt\r\n");
            EnterState(CAR_STATE_PRETILT, nowMs);
            return;
        }

        if ((uint32_t)(nowMs - g_balanceReadyWaitStartMs) >=
            CAR_BALANCE_READY_TIMEOUT_MS)
        {
            Serial_SendString(
                "BALANCE_ACK timeout: level/camera/controller/target not ready\r\n");
            EnterFault(CAR_FAULT_BALANCE_NOT_READY, nowMs);
        }
        return;
    }

    if (StableTimer_Update(
            &g_startMarkerTimer,
            IsMarkerCandidate(),
            nowMs,
            CAR_START_MARKER_STABLE_MS))
    {
        g_finishMarkerArmed = false;
        Serial_SendString("START_MARKER detected\r\n");

        if (MissionIsBalance(g_status.missionMode))
        {
            g_balanceMarkerConfirmed = true;
            g_balanceReadyWaitStartMs = nowMs;
            Serial_SendString(
                "WAIT_BALANCE_ACK: matching task must be ready before launch\r\n");
        }
        else
        {
            EnterState(CAR_STATE_RUNNING, nowMs);
        }
        return;
    }

#if CAR_START_MARKER_TIMEOUT_MS > 0U
    if (g_status.stateElapsedMs >= CAR_START_MARKER_TIMEOUT_MS)
    {
        EnterFault(CAR_FAULT_START_NO_LINE, nowMs);
    }
#endif
}

static void UpdatePreTilt(uint32_t nowMs)
{
    uint8_t missionCode = MissionLinkCode(g_status.missionMode);

    if (g_status.stateElapsedMs >= CAR_BALANCE_PRETILT_MIN_MS &&
        CarMotionLink_IsPreTiltReady(missionCode, nowMs))
    {
        Serial_SendString(
            "PRETILT ready; start motion planner\r\n");
        EnterState(CAR_STATE_RUNNING, nowMs);
        return;
    }

    if (g_status.stateElapsedMs >= CAR_BALANCE_PRETILT_MAX_MS)
    {
        if (CarMotionLink_IsBalanceReady(missionCode, nowMs))
        {
            Serial_SendString(
                "PRETILT max wait reached; launch with current compensation\r\n");
            EnterState(CAR_STATE_RUNNING, nowMs);
        }
        else
        {
            Serial_SendString(
                "PRETILT failed: balance ACK lost or faulted\r\n");
            EnterFault(CAR_FAULT_BALANCE_NOT_READY, nowMs);
        }
    }
}

static bool UpdateLineLoss(uint32_t nowMs)
{
    if (StableTimer_Update(
            &g_lineLostTimer,
            !g_status.lineValid,
            nowMs,
            CAR_LINE_LOST_FAULT_MS))
    {
        EnterFault(CAR_FAULT_LINE_LOST, nowMs);
        return true;
    }
    return false;
}

static void FinishMarkerDetected(uint32_t nowMs)
{
    /* This function is reached only by requirement 2. */
    g_status.lapCompleted = true;
    Serial_SendString("FINISH_MARKER detected STOP_A\r\n");
    g_stopToFinished = true;
    EnterState(CAR_STATE_STOPPING, nowMs);
}

static void UpdateRunning(uint32_t nowMs)
{
    if (g_straightTuning)
    {
        return;
    }

    /*
     * Requirements 4/5/6 use one continuous program. Once started, marker
     * patterns and elapsed time never request a stop or a speed change. The
     * normal line-loss safety supervision remains active.
     */
    if (MissionIsBalance(g_status.missionMode))
    {
        (void)UpdateLineLoss(nowMs);
        return;
    }

    /* Requirement 2 logic below is intentionally unchanged. */

    /* The start line must be left before the same wide pattern may finish. */
    if (!g_finishMarkerArmed)
    {
        if (StableTimer_Update(
                &g_markerLeaveTimer,
                g_status.lineValid && !IsMarkerCandidate(),
                nowMs,
                CAR_MARKER_LEAVE_STABLE_MS))
        {
            g_finishMarkerArmed = true;
            Serial_SendString("FINISH_MARKER armed\r\n");
        }
    }
    else if ((g_status.driveTimeMs >= CAR_FINISH_MIN_DRIVE_MS) &&
             IsStrongMarker())
    {
        /* A marker observed at the deadline is accepted before timeout. */
        FinishMarkerDetected(nowMs);
        return;
    }

    if (g_status.driveTimeMs >= CAR_REQ2_NO_MARKER_STOP_MS)
    {
        g_stopToFinished = true;
        Serial_SendString("REQ2 timeout: no finish marker at 20s\r\n");
        EnterState(CAR_STATE_STOPPING, nowMs);
        return;
    }

    (void)UpdateLineLoss(nowMs);
}

static void UpdateStopping(uint32_t nowMs)
{
    if (StableTimer_Update(
            &g_stopTimer,
            WheelsStopped(),
            nowMs,
            CAR_STOP_STABLE_MS) ||
        (g_status.stateElapsedMs >= CAR_STOP_TIMEOUT_MS))
    {
        EnterState(
            g_stopToFinished ? CAR_STATE_FINISHED : CAR_STATE_IDLE,
            nowMs);
    }
}

static void UpdateStateMachine(uint32_t nowMs)
{
    g_status.stateElapsedMs = (uint32_t)(nowMs - g_stateEnterMs);

    switch (g_status.state)
    {
        case CAR_STATE_ARMING: UpdateArming(nowMs); break;
        case CAR_STATE_PRETILT: UpdatePreTilt(nowMs); break;
        case CAR_STATE_RUNNING: UpdateRunning(nowMs); break;
        case CAR_STATE_STOPPING: UpdateStopping(nowMs); break;
        case CAR_STATE_IDLE:
        case CAR_STATE_FINISHED:
        case CAR_STATE_FAULT:
        default:
            break;
    }
}

static void ApplyControl(uint32_t nowMs)
{
    if (g_status.state == CAR_STATE_RUNNING)
    {
        if (g_straightTuning)
        {
            int32_t plannedBaseCps;

            MotionPlanner_SetTargetCps(g_status.baseCps);
            plannedBaseCps = MotionPlanner_Update(nowMs);
            LineFollow_SetDirectWheelTargets(plannedBaseCps, plannedBaseCps);
            SpeedPI_ControlTask(nowMs);
        }
        else
        {
            int32_t activeBase = g_status.baseCps;

            /*
             * Requirements 4/5/6 must actually use the same planner whose
             * acceleration is transmitted to the balance board. Previously
             * LineFollow jumped directly to 1500 CPS while the transmitted
             * planner acceleration could remain zero.
             */
            if (MissionIsBalance(g_status.missionMode))
            {
                MotionPlanner_SetTargetCps(g_status.baseCps);
                activeBase = MotionPlanner_Update(nowMs);
            }
            else if ((g_status.driveTimeMs >= CAR_REQ2_SLOWDOWN_START_MS) &&
                     (activeBase > CAR_REQ2_SLOWDOWN_CPS))
            {
                activeBase = CAR_REQ2_SLOWDOWN_CPS;
            }

            LineFollow_SetBaseCps(activeBase);
            LineFollow_ControlTask(nowMs);
        }
    }
    else
    {
        /* Keep speed estimates updated while active braking is applied. */
        SpeedPI_ControlTask(nowMs);
    }
}

void CarControl_Init(void)
{
    g_status = (CarControlStatus_t){0};
    g_status.state = CAR_STATE_IDLE;
    g_status.fault = CAR_FAULT_NONE;
    g_status.missionMode = CAR_MISSION_STOP_AT_A;
    g_status.linePattern = GRAY_PATTERN_LOST;
    g_status.baseCps = MissionDefaultBase(g_status.missionMode);

    g_stateEnterMs = 0U;
    g_lastSupervisorMs = 0U;
    g_driveStartMs = 0U;
    g_timeInitialized = false;
    g_straightTuning = false;
    g_stopToFinished = false;
    g_finishMarkerArmed = false;
    g_balanceMarkerConfirmed = false;
    g_balanceReadyWaitStartMs = 0U;

    ResetStableTimers();
    DriveTimer_Reset();
    MissionApplyMotionProfile(g_status.missionMode);
    LineFollow_SetBaseCps(g_status.baseCps);
    LineFollow_Disable();
    SpeedPI_StopAll();

    Serial_SendString("CarControl H2026 missions 2 / 45-center / 6-captured-target\r\n");
}

bool CarControl_Start(uint32_t nowMs)
{
    return CarControl_StartMission(nowMs, 0U);
}

bool CarControl_StartMode(uint32_t nowMs, CarMissionMode_t mode)
{
    if (!CarControl_SetMissionMode(mode))
    {
        return false;
    }
    return CarControl_Start(nowMs);
}

bool CarControl_StartMission(uint32_t nowMs, uint16_t targetIntersections)
{
    (void)targetIntersections;

    if (g_status.state != CAR_STATE_IDLE)
    {
        return false;
    }

    g_straightTuning = false;
    g_status.fault = CAR_FAULT_NONE;
    g_status.intersectionCount = 0U;
    g_status.missionIntersectionTarget = 0U;
    g_status.lastLinePosition = 0;
    g_status.finishPassElapsedMs = 0U;
    g_status.lapCompleted = false;
    g_stopToFinished = false;
    g_finishMarkerArmed = false;
    g_balanceMarkerConfirmed = false;
    g_balanceReadyWaitStartMs = 0U;
    DriveTimer_Reset();

    /* Competition timing starts at the physical/serial start command. */
    DriveTimer_Start(nowMs);
    EnterState(CAR_STATE_ARMING, nowMs);
    return true;
}

bool CarControl_StartStraightTuning(uint32_t nowMs, int32_t targetCps)
{
    if ((targetCps < CAR_MIN_BASE_CPS) ||
        (targetCps > SPEED_PI_TARGET_LIMIT_CPS))
    {
        return false;
    }

    if ((g_status.state != CAR_STATE_IDLE) &&
        (g_status.state != CAR_STATE_FAULT) &&
        (g_status.state != CAR_STATE_FINISHED))
    {
        return false;
    }

    g_straightTuning = true;
    MotionPlanner_SetProfile(
        MOTION_REQ2_ACCEL_MM_S2,
        MOTION_REQ2_DECEL_MM_S2);
    g_status.fault = CAR_FAULT_NONE;
    g_status.baseCps = targetCps;
    g_status.intersectionCount = 0U;
    g_status.missionIntersectionTarget = 0U;
    g_status.lastLinePosition = 0;
    g_status.finishPassElapsedMs = 0U;
    g_status.lapCompleted = false;
    g_finishMarkerArmed = false;
    g_stopToFinished = false;
    DriveTimer_Reset();
    DriveTimer_Start(nowMs);
    EnterState(CAR_STATE_RUNNING, nowMs);
    return true;
}

bool CarControl_Stop(uint32_t nowMs)
{
    if ((g_status.state == CAR_STATE_IDLE) ||
        (g_status.state == CAR_STATE_FAULT) ||
        (g_status.state == CAR_STATE_FINISHED))
    {
        SpeedPI_StopAll();
        return true;
    }

    DriveTimer_Stop(nowMs);
    g_stopToFinished = false;
    EnterState(CAR_STATE_STOPPING, nowMs);
    return true;
}

bool CarControl_Reset(uint32_t nowMs)
{
    if ((g_status.state != CAR_STATE_IDLE) &&
        (g_status.state != CAR_STATE_FAULT) &&
        (g_status.state != CAR_STATE_FINISHED))
    {
        return false;
    }

    g_status.fault = CAR_FAULT_NONE;
    g_status.intersectionCount = 0U;
    g_status.missionIntersectionTarget = 0U;
    g_status.finishPassElapsedMs = 0U;
    g_status.lapCompleted = false;
    g_stopToFinished = false;
    g_finishMarkerArmed = false;
    DriveTimer_Reset();
    EnterState(CAR_STATE_IDLE, nowMs);
    return true;
}

bool CarControl_SetMissionMode(CarMissionMode_t mode)
{
    if ((mode != CAR_MISSION_STOP_AT_A) &&
        (mode != CAR_MISSION_BALANCE_CENTER) &&
        (mode != CAR_MISSION_BALANCE_CAPTURED_TARGET))
    {
        return false;
    }

    if ((g_status.state != CAR_STATE_IDLE) &&
        (g_status.state != CAR_STATE_FINISHED) &&
        (g_status.state != CAR_STATE_FAULT))
    {
        return false;
    }

    g_status.missionMode = mode;
    g_status.baseCps = MissionDefaultBase(mode);
    MissionApplyMotionProfile(mode);
    LineFollow_SetBaseCps(g_status.baseCps);

    Serial_SendString("MISSION:");
    Serial_SendString(MissionName(mode));
    Serial_SendString("\r\n");
    return true;
}

bool CarControl_ToggleMissionMode(void)
{
    CarMissionMode_t next =
        (g_status.missionMode == CAR_MISSION_STOP_AT_A) ?
        CAR_MISSION_BALANCE_CENTER : CAR_MISSION_STOP_AT_A;

    return CarControl_SetMissionMode(next);
}

CarMissionMode_t CarControl_GetMissionMode(void)
{
    return g_status.missionMode;
}

bool CarControl_IsStraightTuning(void)
{
    return g_straightTuning;
}

bool CarControl_IsBalanceMission(void)
{
    return MissionIsBalance(g_status.missionMode);
}

bool CarControl_IsRequirement6(void)
{
    return g_status.missionMode ==
        CAR_MISSION_BALANCE_CAPTURED_TARGET;
}

bool CarControl_SetBaseCps(int32_t baseCps)
{
    if ((baseCps < CAR_MIN_BASE_CPS) || (baseCps > CAR_MAX_BASE_CPS))
    {
        return false;
    }

    g_status.baseCps = baseCps;
    LineFollow_SetBaseCps(baseCps);
    return true;
}

CarControlStatus_t CarControl_GetStatus(void)
{
    return g_status;
}

uint32_t CarControl_GetDriveTimeMs(void)
{
    return g_status.driveTimeMs;
}

bool CarControl_IsDriveTimerRunning(void)
{
    return g_status.driveTimerRunning;
}

void CarControl_PrintStatus(void)
{
    SpeedPIStatus_t speed = SpeedPI_GetStatus();
    LineFollowStatus_t line = LineFollow_GetStatus();

    Serial_SendString("CAR state=");
    Serial_SendString(StateName(g_status.state));
    Serial_SendString(" mission=");
    Serial_SendString(MissionName(g_status.missionMode));
    Serial_SendString(" mode=");
    Serial_SendString(g_straightTuning ? "STRAIGHT" : "OVAL");
    Serial_SendString(" fault=");
    Serial_SendString(FaultName(g_status.fault));
    Serial_SendString(" mask=");
    DebugPrint_Mask8(g_status.sensorMask);
    Serial_SendString(" markerCount=");
    DebugPrint_U32((uint32_t)MarkerTotalBlackCount());
    Serial_SendString(" markerStrong=");
    DebugPrint_Bool(IsStrongMarker());
    Serial_SendString(" finishArmed=");
    DebugPrint_Bool(g_finishMarkerArmed);
    Serial_SendString(" lap=");
    DebugPrint_Bool(g_status.lapCompleted);
    Serial_SendString(" pat=");
    Serial_SendString(PatternName(g_status.linePattern));
    Serial_SendString(" pos=");
    DebugPrint_I32(g_status.linePosition, true);
    Serial_SendString(" replay/lostMs=");
    DebugPrint_Bool(line.replayingLastState);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_U32(line.lineLostElapsedMs);
    Serial_SendString(" L/R=");
    DebugPrint_I32(line.leftTargetCps, true);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_I32(line.rightTargetCps, true);
    Serial_SendString(" M1/M2=");
    DebugPrint_I32(speed.motor1.filteredCps, true);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_I32(speed.motor2.filteredCps, true);
    Serial_SendString(" stateTime=");
    DebugPrint_U32(g_status.stateElapsedMs);
    Serial_SendString("ms driveTime=");
    DebugPrint_U32(g_status.driveTimeMs);
    Serial_SendString("ms driveRun=");
    DebugPrint_Bool(g_status.driveTimerRunning);
    Serial_SendString("\r\n");
}

void CarControl_Task(uint32_t nowMs)
{
    if (!g_timeInitialized)
    {
        g_stateEnterMs = nowMs;
        g_lastSupervisorMs = nowMs;
        g_timeInitialized = true;
    }

    /* Keep the displayed/finish time current before finish-marker decisions. */
    DriveTimer_Update(nowMs);

    if (TaskDue(
            nowMs,
            &g_lastSupervisorMs,
            CAR_SUPERVISOR_PERIOD_MS))
    {
        if (!g_straightTuning)
        {
            ReadLine();
        }

        UpdateStateMachine(nowMs);
    }

    ApplyControl(nowMs);
}
