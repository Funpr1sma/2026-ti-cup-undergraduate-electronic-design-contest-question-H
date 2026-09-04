#ifndef HARDWARE_MOTION_PLANNER_H_
#define HARDWARE_MOTION_PLANNER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t requestedCps;
    int32_t plannedCps;
    int32_t accelerationMmS2;
    int32_t accelerationLimitMmS2;
    int32_t decelerationLimitMmS2;
    bool updated;
} MotionPlannerStatus_t;

void MotionPlanner_Init(uint32_t nowMs);
void MotionPlanner_SetTargetCps(int32_t targetCps);
void MotionPlanner_SetProfile(int32_t accelerationMmS2, int32_t decelerationMmS2);
int32_t MotionPlanner_Update(uint32_t nowMs);
void MotionPlanner_Reset(uint32_t nowMs);
MotionPlannerStatus_t MotionPlanner_GetStatus(void);

#endif