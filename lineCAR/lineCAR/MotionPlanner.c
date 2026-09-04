#include "MotionPlanner.h"

#include "Config/CarConfig.h"

#include <stdint.h>

static MotionPlannerStatus_t g_motion;
static uint32_t g_lastUpdateMs;

static int32_t CalculateStep(int32_t accelerationMmS2, uint32_t elapsedMs)
{
    int32_t step = (int32_t)(
        (int64_t)accelerationMmS2 *
        CAR_ENCODER_COUNTS_PER_METER *
        elapsedMs / 1000000LL);

    return (step < 1) ? 1 : step;
}

void MotionPlanner_Init(uint32_t nowMs)
{
    g_motion = (MotionPlannerStatus_t){0};
    g_motion.accelerationLimitMmS2 = MOTION_REQ2_ACCEL_MM_S2;
    g_motion.decelerationLimitMmS2 = MOTION_REQ2_DECEL_MM_S2;
    g_lastUpdateMs = nowMs;
}

void MotionPlanner_SetTargetCps(int32_t targetCps)
{
    g_motion.requestedCps = targetCps;
}

void MotionPlanner_SetProfile(int32_t accelerationMmS2, int32_t decelerationMmS2)
{
    if (accelerationMmS2 < 1) {
        accelerationMmS2 = 1;
    }
    if (decelerationMmS2 < 1) {
        decelerationMmS2 = 1;
    }

    g_motion.accelerationLimitMmS2 = accelerationMmS2;
    g_motion.decelerationLimitMmS2 = decelerationMmS2;
}

int32_t MotionPlanner_Update(uint32_t nowMs)
{
    uint32_t elapsedMs;
    int32_t previous;
    int32_t step;

    g_motion.updated = false;
    elapsedMs = (uint32_t)(nowMs - g_lastUpdateMs);

    if (elapsedMs < MOTION_PLANNER_PERIOD_MS)
    {
        return g_motion.plannedCps;
    }
    elapsedMs = MOTION_PLANNER_PERIOD_MS;
    g_lastUpdateMs = nowMs;
    previous = g_motion.plannedCps;

    if (g_motion.plannedCps < g_motion.requestedCps)
    {
        step = CalculateStep(g_motion.accelerationLimitMmS2, elapsedMs);

        if ((g_motion.requestedCps - g_motion.plannedCps) <= step)
            g_motion.plannedCps = g_motion.requestedCps;
        else
            g_motion.plannedCps += step;
    }
    else if (g_motion.plannedCps > g_motion.requestedCps)
    {
        step = CalculateStep(g_motion.decelerationLimitMmS2, elapsedMs);

        if ((g_motion.plannedCps - g_motion.requestedCps) <= step)
            g_motion.plannedCps = g_motion.requestedCps;
        else
            g_motion.plannedCps -= step;
    }

    g_motion.accelerationMmS2 = (int32_t)(
        (int64_t)(g_motion.plannedCps - previous) *
        1000000LL /
        ((int64_t)CAR_ENCODER_COUNTS_PER_METER * elapsedMs));

    g_motion.updated = true;
    return g_motion.plannedCps;
}

void MotionPlanner_Reset(uint32_t nowMs)
{
    g_motion.requestedCps = 0;
    g_motion.plannedCps = 0;
    g_motion.accelerationMmS2 = 0;
    g_motion.updated = true;
    g_lastUpdateMs = nowMs;
}

MotionPlannerStatus_t MotionPlanner_GetStatus(void)
{
    return g_motion;
}