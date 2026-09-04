#ifndef HARDWARE_CAR_MOTION_LINK_H_
#define HARDWARE_CAR_MOTION_LINK_H_

#include <stdbool.h>
#include <stdint.h>

/* Car -> balance 9-byte frame. */
#define CAR_MOTION_FLAG_RUNNING       (1U << 0)
#define CAR_MOTION_FLAG_EMERGENCY     (1U << 1)
#define CAR_MOTION_FLAG_READY         (1U << 2)
#define CAR_MOTION_FLAG_BALANCE_MODE  (1U << 3)
#define CAR_MOTION_FLAG_PRETILT       (1U << 4)
#define CAR_MOTION_FLAG_REQUIREMENT6  (1U << 5)

#define CAR_MOTION_MISSION_NONE       (0U)
#define CAR_MOTION_MISSION_CENTER     (45U)
#define CAR_MOTION_MISSION_CAPTURED   (6U)

/* Balance -> car acknowledgement status bits. */
#define BALANCE_ACK_LEVEL_READY       (1U << 0)
#define BALANCE_ACK_CAMERA_VALID      (1U << 1)
#define BALANCE_ACK_CONTROL_ACTIVE    (1U << 2)
#define BALANCE_ACK_TARGET_LATCHED    (1U << 3)
#define BALANCE_ACK_FAULT             (1U << 4)
#define BALANCE_ACK_PRETILT_READY     (1U << 5)

#define BALANCE_ACK_REQUIRED_BITS \
    (BALANCE_ACK_LEVEL_READY | BALANCE_ACK_CAMERA_VALID | \
     BALANCE_ACK_CONTROL_ACTIVE | BALANCE_ACK_TARGET_LATCHED)

typedef struct
{
    uint32_t sentFrameCount;
    uint32_t droppedFrameCount;
    uint32_t sentByteCount;
    uint8_t sequence;
    uint8_t txBusy;
    uint8_t txIndex;

    uint32_t ackFrameCount;
    uint32_t ackChecksumErrorCount;
    uint32_t ackByteCount;
    uint32_t ackAgeMs;
    uint8_t ackSequence;
    uint8_t ackStatus;
    uint8_t ackMission;
    uint8_t ackError;
    uint8_t ackValid;
} CarMotionLinkStatus_t;

void CarMotionLink_Init(uint32_t nowMs);

/*
 * Full-duplex non-blocking service. PB6 transmits the car frame and PB7 polls
 * the balance-board acknowledgement. Neither direction spin-waits.
 */
void CarMotionLink_Task(
    uint32_t nowMs,
    bool ready,
    bool running,
    bool emergency,
    bool preTilt,
    bool balanceMode,
    bool requirement6
);

bool CarMotionLink_IsBalanceReady(uint8_t missionCode, uint32_t nowMs);
bool CarMotionLink_IsPreTiltReady(uint8_t missionCode, uint32_t nowMs);
CarMotionLinkStatus_t CarMotionLink_GetStatus(void);

#endif /* HARDWARE_CAR_MOTION_LINK_H_ */
