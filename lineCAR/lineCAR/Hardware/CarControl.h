#ifndef HARDWARE_CAR_CONTROL_H_
#define HARDWARE_CAR_CONTROL_H_

#include "GraySensor.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAR_STATE_IDLE = 0,
    CAR_STATE_ARMING,
    CAR_STATE_PRETILT,
    CAR_STATE_RUNNING,
    CAR_STATE_PASSING_FINISH,
    CAR_STATE_STOPPING,
    CAR_STATE_FINISHED,
    CAR_STATE_FAULT
} CarState_t;

typedef enum
{
    CAR_FAULT_NONE = 0,
    CAR_FAULT_START_NO_LINE,
    CAR_FAULT_LINE_LOST,
    CAR_FAULT_BALANCE_NOT_READY
} CarFault_t;

/*
 * Integrated competition mission programs:
 *   2  - PB14: fast lap and stop at A.
 *   45 - PB11: requirements 4/5 share one continuous center-hold program.
 *   6  - PB10: continuous program; the balance board captures the current
 *        camera ball-center coordinate as the target when the start request
 *        is received.
 */
typedef enum
{
    CAR_MISSION_STOP_AT_A = 2,
    CAR_MISSION_BALANCE_CENTER = 45,
    CAR_MISSION_BALANCE_CAPTURED_TARGET = 6
} CarMissionMode_t;

typedef struct
{
    CarState_t state;
    CarFault_t fault;
    CarMissionMode_t missionMode;

    bool lineValid;
    uint8_t sensorMask;
    uint8_t activeCount;
    GraySensorPattern_t linePattern;
    int32_t linePosition;
    int32_t lastLinePosition;

    int32_t baseCps;

    /* Kept as zero-valued compatibility fields for the existing VOFA frame. */
    uint16_t intersectionCount;
    uint16_t missionIntersectionTarget;

    uint32_t stateElapsedMs;
    uint32_t driveTimeMs;
    uint32_t finishPassElapsedMs;
    bool driveTimerRunning;
    bool lapCompleted;
} CarControlStatus_t;

void CarControl_Init(void);

/* Start the currently selected mission. Physical buttons normally call StartMode(). */
bool CarControl_Start(uint32_t nowMs);
bool CarControl_StartMode(uint32_t nowMs, CarMissionMode_t mode);

/* Compatibility entry point retained for existing VOFA scripts. */
bool CarControl_StartMission(uint32_t nowMs, uint16_t targetIntersections);
bool CarControl_StartStraightTuning(uint32_t nowMs, int32_t targetCps);
bool CarControl_Stop(uint32_t nowMs);
bool CarControl_Reset(uint32_t nowMs);

bool CarControl_SetMissionMode(CarMissionMode_t mode);
bool CarControl_ToggleMissionMode(void);
CarMissionMode_t CarControl_GetMissionMode(void);

bool CarControl_IsStraightTuning(void);
bool CarControl_IsBalanceMission(void);
bool CarControl_IsRequirement6(void);
bool CarControl_SetBaseCps(int32_t baseCps);

CarControlStatus_t CarControl_GetStatus(void);
uint32_t CarControl_GetDriveTimeMs(void);
bool CarControl_IsDriveTimerRunning(void);
void CarControl_PrintStatus(void);
void CarControl_Task(uint32_t nowMs);

#endif /* HARDWARE_CAR_CONTROL_H_ */
