#ifndef HARDWARE_SPEED_PI_H_
#define HARDWARE_SPEED_PI_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SPEED_PI_MOTOR1 = 0,
    SPEED_PI_MOTOR2 = 1
} SpeedPIWheel_t;

#define SPEED_PI_KP_Q12_MIN    (0L)
#define SPEED_PI_KP_Q12_MAX    (1024L)
#define SPEED_PI_KI_Q12_MIN    (0L)
#define SPEED_PI_KI_Q12_MAX    (64L)

typedef struct
{
    int32_t kpQ12;
    int32_t kiQ12;
} SpeedPIGains_t;

typedef struct
{
    bool enabled;
    int32_t targetCps;
    int32_t rawCps;
    int32_t filteredCps;
    int32_t errorCps;

    /* Individual PI terms after conversion to PWM percent. */
    int16_t proportionalPercent;
    int16_t integralPercent;
    int16_t outputPercent;

    bool saturated;
} SpeedPIWheelStatus_t;

typedef struct
{
    SpeedPIWheelStatus_t motor1;
    SpeedPIWheelStatus_t motor2;
} SpeedPIStatus_t;

void SpeedPI_Init(void);

void SpeedPI_SetTarget(SpeedPIWheel_t wheel, int32_t targetCps);
void SpeedPI_SetTargets(int32_t motor1TargetCps, int32_t motor2TargetCps);

void SpeedPI_StopWheel(SpeedPIWheel_t wheel);
void SpeedPI_StopAll(void);

void SpeedPI_ResetIntegral(SpeedPIWheel_t wheel);
void SpeedPI_ResetAllIntegrals(void);

/*
 * Set a temporary minimum PWM used only while a commanded wheel is nearly
 * stopped or still moving opposite to its target. Pass 0 to disable it.
 */
void SpeedPI_SetStartupMinimumPercent(
    SpeedPIWheel_t wheel,
    uint8_t percent
);
void SpeedPI_SetAllStartupMinimumPercent(uint8_t percent);

bool SpeedPI_SetGains(
    SpeedPIWheel_t wheel,
    int32_t kpQ12,
    int32_t kiQ12
);

SpeedPIGains_t SpeedPI_GetGains(SpeedPIWheel_t wheel);
SpeedPIWheelStatus_t SpeedPI_GetWheelStatus(SpeedPIWheel_t wheel);
SpeedPIStatus_t SpeedPI_GetStatus(void);

void SpeedPI_PrintStatus(void);

/* Run encoder sampling and both PI controllers. No serial command handling. */
void SpeedPI_ControlTask(uint32_t nowMs);

#endif /* HARDWARE_SPEED_PI_H_ */
