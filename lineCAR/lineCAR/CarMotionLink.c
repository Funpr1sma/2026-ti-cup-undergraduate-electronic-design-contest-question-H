#include "CarMotionLink.h"

#include "Config/CarConfig.h"
#include "MotionPlanner.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define CAR_MOTION_FRAME_SIZE        (9U)
#define CAR_MOTION_FRAME_HEAD_0      (0xA5U)
#define CAR_MOTION_FRAME_HEAD_1      (0x5AU)

#define BALANCE_ACK_FRAME_SIZE       (7U)
#define BALANCE_ACK_FRAME_HEAD_0     (0x5AU)
#define BALANCE_ACK_FRAME_HEAD_1     (0xA5U)
#define BALANCE_ACK_TIMEOUT_MS       (120U)
#define BALANCE_ACK_RX_DRAIN_LIMIT   (32U)

static uint8_t g_sequence;
static uint32_t g_lastSendMs;
static uint32_t g_lastTaskNowMs;
static uint32_t g_sentFrameCount;
static uint32_t g_txDropCount;
static uint32_t g_sentByteCount;
static uint8_t g_txFrame[CAR_MOTION_FRAME_SIZE];
static uint8_t g_txIndex;
static bool g_txBusy;

static uint8_t g_ackBuffer[BALANCE_ACK_FRAME_SIZE];
static uint8_t g_ackIndex;
static uint8_t g_ackSequence;
static uint8_t g_ackStatus;
static uint8_t g_ackMission;
static uint8_t g_ackError;
static uint32_t g_ackLastFrameMs;
static uint32_t g_ackFrameCount;
static uint32_t g_ackChecksumErrorCount;
static uint32_t g_ackByteCount;

static void WriteI16(uint8_t *out, int32_t value)
{
    uint16_t raw;

    if (value > 32767)
    {
        value = 32767;
    }
    else if (value < -32768)
    {
        value = -32768;
    }

    raw = (uint16_t)(int16_t)value;
    out[0] = (uint8_t)raw;
    out[1] = (uint8_t)(raw >> 8U);
}

static uint8_t CalculateChecksum(const uint8_t *frame, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t index;

    for (index = 0U; index < length; index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum;
}

static void ServiceTx(void)
{
    while (g_txBusy && (g_txIndex < CAR_MOTION_FRAME_SIZE))
    {
        if (!DL_UART_Main_transmitDataCheck(
                UART_1_INST,
                g_txFrame[g_txIndex]))
        {
            return;
        }

        g_txIndex++;
        g_sentByteCount++;
    }

    if (g_txBusy && (g_txIndex >= CAR_MOTION_FRAME_SIZE))
    {
        g_txBusy = false;
        g_txIndex = 0U;
        g_sentFrameCount++;
    }
}

static void AcceptAckFrame(const uint8_t *frame, uint32_t nowMs)
{
    g_ackSequence = frame[2];
    g_ackStatus = frame[3];
    g_ackMission = frame[4];
    g_ackError = frame[5];
    g_ackLastFrameMs = nowMs;
    g_ackFrameCount++;
}

static void ProcessAckByte(uint8_t byte, uint32_t nowMs)
{
    g_ackByteCount++;

    if (g_ackIndex == 0U)
    {
        if (byte == BALANCE_ACK_FRAME_HEAD_0)
        {
            g_ackBuffer[0] = byte;
            g_ackIndex = 1U;
        }
        return;
    }

    if (g_ackIndex == 1U)
    {
        if (byte == BALANCE_ACK_FRAME_HEAD_1)
        {
            g_ackBuffer[1] = byte;
            g_ackIndex = 2U;
        }
        else if (byte == BALANCE_ACK_FRAME_HEAD_0)
        {
            g_ackBuffer[0] = byte;
            g_ackIndex = 1U;
        }
        else
        {
            g_ackIndex = 0U;
        }
        return;
    }

    g_ackBuffer[g_ackIndex++] = byte;
    if (g_ackIndex < BALANCE_ACK_FRAME_SIZE)
    {
        return;
    }

    g_ackIndex = 0U;
    if (CalculateChecksum(g_ackBuffer, BALANCE_ACK_FRAME_SIZE - 1U) !=
        g_ackBuffer[BALANCE_ACK_FRAME_SIZE - 1U])
    {
        g_ackChecksumErrorCount++;
        return;
    }

    AcceptAckFrame(g_ackBuffer, nowMs);
}

static void ServiceRx(uint32_t nowMs)
{
    uint32_t drained = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST) &&
           drained < BALANCE_ACK_RX_DRAIN_LIMIT)
    {
        ProcessAckByte(DL_UART_Main_receiveData(UART_1_INST), nowMs);
        drained++;
    }
}

static void BuildFrame(
    bool ready,
    bool running,
    bool emergency,
    bool preTilt,
    bool balanceMode,
    bool requirement6)
{
    MotionPlannerStatus_t motion = MotionPlanner_GetStatus();
    uint8_t flags = 0U;
    int32_t accelerationMmS2 = motion.accelerationMmS2;

    if (emergency)
    {
        flags = CAR_MOTION_FLAG_EMERGENCY;
        accelerationMmS2 = 0;
    }
    else if (running)
    {
        flags = CAR_MOTION_FLAG_RUNNING;
    }
    else if (preTilt)
    {
        /* Wheels remain stopped. Send the known launch acceleration early so
         * the balance board can establish the compensating screw angle. */
        flags = CAR_MOTION_FLAG_READY | CAR_MOTION_FLAG_PRETILT;
        accelerationMmS2 = MOTION_BALANCE_ACCEL_MM_S2;
    }
    else if (ready)
    {
        flags = CAR_MOTION_FLAG_READY;
        accelerationMmS2 = 0;
    }
    else
    {
        accelerationMmS2 = 0;
    }

    if (balanceMode)
    {
        flags |= CAR_MOTION_FLAG_BALANCE_MODE;
    }
    if (requirement6)
    {
        flags |= CAR_MOTION_FLAG_REQUIREMENT6;
    }

    g_txFrame[0] = CAR_MOTION_FRAME_HEAD_0;
    g_txFrame[1] = CAR_MOTION_FRAME_HEAD_1;
    g_txFrame[2] = g_sequence++;
    g_txFrame[3] = flags;
    WriteI16(&g_txFrame[4], motion.plannedCps);
    WriteI16(&g_txFrame[6], accelerationMmS2);
    g_txFrame[8] = CalculateChecksum(g_txFrame, 8U);
    g_txIndex = 0U;
    g_txBusy = true;
}

void CarMotionLink_Init(uint32_t nowMs)
{
    g_sequence = 0U;
    g_lastSendMs = nowMs;
    g_lastTaskNowMs = nowMs;
    g_sentFrameCount = 0U;
    g_txDropCount = 0U;
    g_sentByteCount = 0U;
    g_txIndex = 0U;
    g_txBusy = false;

    g_ackIndex = 0U;
    g_ackSequence = 0U;
    g_ackStatus = 0U;
    g_ackMission = CAR_MOTION_MISSION_NONE;
    g_ackError = 0U;
    g_ackLastFrameMs = 0U;
    g_ackFrameCount = 0U;
    g_ackChecksumErrorCount = 0U;
    g_ackByteCount = 0U;
}

void CarMotionLink_Task(
    uint32_t nowMs,
    bool ready,
    bool running,
    bool emergency,
    bool preTilt,
    bool balanceMode,
    bool requirement6)
{
    g_lastTaskNowMs = nowMs;
    ServiceRx(nowMs);
    ServiceTx();

    if ((uint32_t)(nowMs - g_lastSendMs) < CAR_MOTION_LINK_PERIOD_MS)
    {
        return;
    }

    g_lastSendMs += CAR_MOTION_LINK_PERIOD_MS;
    if ((uint32_t)(nowMs - g_lastSendMs) >= CAR_MOTION_LINK_PERIOD_MS)
    {
        g_lastSendMs = nowMs;
    }

    if (g_txBusy)
    {
        g_txDropCount++;
        return;
    }

    BuildFrame(
        ready,
        running,
        emergency,
        preTilt,
        balanceMode,
        requirement6);
    ServiceTx();
}

bool CarMotionLink_IsBalanceReady(uint8_t missionCode, uint32_t nowMs)
{
    uint8_t required = BALANCE_ACK_REQUIRED_BITS;

    if (g_ackFrameCount == 0U ||
        (uint32_t)(nowMs - g_ackLastFrameMs) > BALANCE_ACK_TIMEOUT_MS ||
        g_ackMission != missionCode ||
        (g_ackStatus & BALANCE_ACK_FAULT) != 0U)
    {
        return false;
    }

    return (g_ackStatus & required) == required;
}

bool CarMotionLink_IsPreTiltReady(uint8_t missionCode, uint32_t nowMs)
{
    return CarMotionLink_IsBalanceReady(missionCode, nowMs) &&
        ((g_ackStatus & BALANCE_ACK_PRETILT_READY) != 0U);
}

CarMotionLinkStatus_t CarMotionLink_GetStatus(void)
{
    CarMotionLinkStatus_t status;

    status.sentFrameCount = g_sentFrameCount;
    status.droppedFrameCount = g_txDropCount;
    status.sentByteCount = g_sentByteCount;
    status.sequence = g_sequence;
    status.txBusy = g_txBusy ? 1U : 0U;
    status.txIndex = g_txIndex;

    status.ackFrameCount = g_ackFrameCount;
    status.ackChecksumErrorCount = g_ackChecksumErrorCount;
    status.ackByteCount = g_ackByteCount;
    status.ackAgeMs = (g_ackFrameCount == 0U) ? 0xFFFFFFFFU :
        (uint32_t)(g_lastTaskNowMs - g_ackLastFrameMs);
    status.ackSequence = g_ackSequence;
    status.ackStatus = g_ackStatus;
    status.ackMission = g_ackMission;
    status.ackError = g_ackError;
    status.ackValid = (g_ackFrameCount != 0U &&
        status.ackAgeMs <= BALANCE_ACK_TIMEOUT_MS) ? 1U : 0U;
    return status;
}
