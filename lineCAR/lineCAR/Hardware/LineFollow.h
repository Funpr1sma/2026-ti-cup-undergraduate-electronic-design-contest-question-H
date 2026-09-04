#ifndef HARDWARE_LINE_FOLLOW_H_
#define HARDWARE_LINE_FOLLOW_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool enabled;

    /* Raw current-frame validity. */
    bool lineValid;
    uint8_t sensorMask;

    /* True while the controller is replaying the last valid line state. */
    bool replayingLastState;
    uint32_t lineLostElapsedMs;

    int32_t position;
    int32_t error;
    int32_t derivative;
    int32_t correctionCps;

    int32_t baseCps;
    int32_t leftTargetCps;
    int32_t rightTargetCps;
} LineFollowStatus_t;

void LineFollow_Init(void);
void LineFollow_Enable(void);
void LineFollow_Disable(void);

void LineFollow_SetBaseCps(int32_t baseCps);
bool LineFollow_SetGains(int32_t kpNum, int32_t kdNum);
void LineFollow_GetGains(int32_t *kpNum, int32_t *kdNum);

/* Direct targets are expressed as physical left/right wheel speed. */
void LineFollow_SetDirectWheelTargets(
    int32_t leftTargetCps,
    int32_t rightTargetCps
);

LineFollowStatus_t LineFollow_GetStatus(void);
void LineFollow_PrintStatus(void);

/* Run line PD when due, then run the speed PI inner loop. */
void LineFollow_ControlTask(uint32_t nowMs);

#endif /* HARDWARE_LINE_FOLLOW_H_ */
