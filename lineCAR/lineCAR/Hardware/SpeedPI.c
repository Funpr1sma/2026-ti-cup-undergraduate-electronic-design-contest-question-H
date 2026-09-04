#include "SpeedPI.h"

#include "Config/CarConfig.h"
#include "DebugPrint.h"
#include "Encoder.h"
#include "Motor.h"
#include "Serial.h"

#include <stdbool.h>
#include <stdint.h>

#define SPEED_PI_WHEEL_COUNT (2U)
#define SPEED_PI_INTEGRAL_LIMIT_Q12 \
    ((int32_t)SPEED_PI_OUTPUT_LIMIT_PERCENT * (int32_t)SPEED_PI_SCALE_Q12)

typedef struct
{
    SpeedPIWheelStatus_t status;
    int32_t proportionalQ12;
    int32_t integralQ12;
    uint8_t startupMinimumPercent;
    bool filterInitialized;
} SpeedPIController_t;

static SpeedPIController_t g_controller[SPEED_PI_WHEEL_COUNT];
static SpeedPIGains_t g_gains[SPEED_PI_WHEEL_COUNT];

static bool IsValidWheel(SpeedPIWheel_t wheel)
{
    return (wheel == SPEED_PI_MOTOR1) || (wheel == SPEED_PI_MOTOR2);
}

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

static MotorId_t MotorForWheel(SpeedPIWheel_t wheel)
{
    return (wheel == SPEED_PI_MOTOR1) ? MOTOR_ID_1 : MOTOR_ID_2;
}

static int32_t RawCpsForWheel(SpeedPIWheel_t wheel)
{
    return (wheel == SPEED_PI_MOTOR1) ?
        Encoder_GetMotor1Cps() : Encoder_GetMotor2Cps();
}

static void ClearController(SpeedPIController_t *controller)
{
    controller->status.enabled = false;
    controller->status.targetCps = 0;
    controller->status.rawCps = 0;
    controller->status.filteredCps = 0;
    controller->status.errorCps = 0;
    controller->status.proportionalPercent = 0;
    controller->status.integralPercent = 0;
    controller->status.outputPercent = 0;
    controller->status.saturated = false;

    controller->proportionalQ12 = 0;
    controller->integralQ12 = 0;
    controller->startupMinimumPercent = 0U;
    controller->filterInitialized = false;
}

static int32_t FeedforwardQ12ForTarget(int32_t targetCps)
{
    return
        targetCps *
        (int32_t)SPEED_PI_FEEDFORWARD_NUM *
        (int32_t)SPEED_PI_SCALE_Q12 /
        (int32_t)SPEED_PI_FEEDFORWARD_DIV;
}

static void UpdateWheel(SpeedPIWheel_t wheel)
{
    SpeedPIController_t *controller = &g_controller[wheel];
    const SpeedPIGains_t *gains = &g_gains[wheel];
    int32_t lowerLimit;
    int32_t upperLimit;
    int32_t feedforwardQ12;
    int32_t candidateIntegral;
    int32_t candidateOutput;
    int32_t outputPercent;

    controller->status.rawCps = RawCpsForWheel(wheel);

    if (!controller->filterInitialized)
    {
        controller->status.filteredCps = controller->status.rawCps;
        controller->filterInitialized = true;
    }
    else
    {
        controller->status.filteredCps +=
            (controller->status.rawCps - controller->status.filteredCps) /
            SPEED_PI_FILTER_DIVISOR;
    }

    if (!controller->status.enabled || (controller->status.targetCps == 0))
    {
        controller->status.errorCps = 0;
        controller->status.proportionalPercent = 0;
        controller->status.integralPercent = 0;
        controller->status.outputPercent = 0;
        controller->status.saturated = false;
        controller->proportionalQ12 = 0;
        controller->integralQ12 = 0;
        Motor_Brake(MotorForWheel(wheel));
        return;
    }

    controller->status.errorCps =
        controller->status.targetCps - controller->status.filteredCps;

    controller->proportionalQ12 =
        controller->status.errorCps * gains->kpQ12;

    feedforwardQ12 = FeedforwardQ12ForTarget(
        controller->status.targetCps
    );

    candidateIntegral = controller->integralQ12 +
        controller->status.errorCps * gains->kiQ12;
    candidateIntegral = LimitI32(
        candidateIntegral,
        -SPEED_PI_INTEGRAL_LIMIT_Q12,
        SPEED_PI_INTEGRAL_LIMIT_Q12
    );

    if (controller->status.targetCps > 0)
    {
        lowerLimit = 0;
        upperLimit = SPEED_PI_OUTPUT_LIMIT_PERCENT;
    }
    else
    {
        lowerLimit = -SPEED_PI_OUTPUT_LIMIT_PERCENT;
        upperLimit = 0;
    }

    candidateOutput =
        feedforwardQ12 + controller->proportionalQ12 + candidateIntegral;
    outputPercent = candidateOutput / SPEED_PI_SCALE_Q12;
    controller->status.saturated = false;

    /* Conditional integration anti-windup, including feed-forward. */
    if (((outputPercent > upperLimit) && (controller->status.errorCps > 0)) ||
        ((outputPercent < lowerLimit) && (controller->status.errorCps < 0)))
    {
        controller->status.saturated = true;
    }
    else
    {
        controller->integralQ12 = candidateIntegral;
    }

    outputPercent =
        (feedforwardQ12 +
         controller->proportionalQ12 +
         controller->integralQ12) /
        SPEED_PI_SCALE_Q12;
    outputPercent = LimitI32(outputPercent, lowerLimit, upperLimit);

    if (controller->startupMinimumPercent > 0U)
    {
        bool movingOpposite =
            ((controller->status.targetCps > 0) &&
             (controller->status.filteredCps <= 0)) ||
            ((controller->status.targetCps < 0) &&
             (controller->status.filteredCps >= 0));
        bool nearlyStopped =
            AbsI32(controller->status.filteredCps) <=
            SPEED_PI_STARTUP_EXIT_CPS;
        int32_t minimumOutput =
            (int32_t)controller->startupMinimumPercent;

        if (movingOpposite || nearlyStopped)
        {
            if ((controller->status.targetCps > 0) &&
                (outputPercent < minimumOutput))
            {
                outputPercent = minimumOutput;
            }
            else if ((controller->status.targetCps < 0) &&
                     (outputPercent > -minimumOutput))
            {
                outputPercent = -minimumOutput;
            }
        }
    }

    if (((outputPercent == upperLimit) || (outputPercent == lowerLimit)) &&
        ((outputPercent != 0) ||
         (controller->status.errorCps > 50) ||
         (controller->status.errorCps < -50)))
    {
        controller->status.saturated = true;
    }

    controller->status.proportionalPercent =
        (int16_t)(controller->proportionalQ12 / SPEED_PI_SCALE_Q12);
    controller->status.integralPercent =
        (int16_t)(controller->integralQ12 / SPEED_PI_SCALE_Q12);
    controller->status.outputPercent = (int16_t)outputPercent;

    Motor_SetSpeed(MotorForWheel(wheel), controller->status.outputPercent);
}

static void PrintWheel(const char *name, SpeedPIWheel_t wheel)
{
    const SpeedPIWheelStatus_t *status = &g_controller[wheel].status;

    Serial_SendString(name);
    Serial_SendString(status->enabled ? ":EN" : ":OFF");
    Serial_SendString(" Kp=");
    DebugPrint_I32(g_gains[wheel].kpQ12, false);
    Serial_SendString(" Ki=");
    DebugPrint_I32(g_gains[wheel].kiQ12, false);
    Serial_SendString(" T=");
    DebugPrint_I32(status->targetCps, true);
    Serial_SendString(" R=");
    DebugPrint_I32(status->rawCps, true);
    Serial_SendString(" F=");
    DebugPrint_I32(status->filteredCps, true);
    Serial_SendString(" E=");
    DebugPrint_I32(status->errorCps, true);
    Serial_SendString(" P=");
    DebugPrint_I32(status->proportionalPercent, true);
    Serial_SendString("% I=");
    DebugPrint_I32(status->integralPercent, true);
    Serial_SendString("% O=");
    DebugPrint_I32(status->outputPercent, true);
    Serial_SendString(status->saturated ? "% SAT" : "%");
}

void SpeedPI_Init(void)
{
    g_gains[SPEED_PI_MOTOR1].kpQ12 = SPEED_PI_DEFAULT_M1_KP_Q12;
    g_gains[SPEED_PI_MOTOR1].kiQ12 = SPEED_PI_DEFAULT_M1_KI_Q12;
    g_gains[SPEED_PI_MOTOR2].kpQ12 = SPEED_PI_DEFAULT_M2_KP_Q12;
    g_gains[SPEED_PI_MOTOR2].kiQ12 = SPEED_PI_DEFAULT_M2_KI_Q12;

    ClearController(&g_controller[SPEED_PI_MOTOR1]);
    ClearController(&g_controller[SPEED_PI_MOTOR2]);
    Motor_BrakeAll();

    Serial_SendString("SpeedPI initialized: M1=");
    DebugPrint_I32(SPEED_PI_DEFAULT_M1_KP_Q12, false);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_I32(SPEED_PI_DEFAULT_M1_KI_Q12, false);
    Serial_SendString(" M2=");
    DebugPrint_I32(SPEED_PI_DEFAULT_M2_KP_Q12, false);
    Serial_SendByte((uint8_t)'/');
    DebugPrint_I32(SPEED_PI_DEFAULT_M2_KI_Q12, false);
    Serial_SendString("\r\n");
}

void SpeedPI_SetTarget(SpeedPIWheel_t wheel, int32_t targetCps)
{
    SpeedPIController_t *controller;
    int32_t previousTarget;
    bool sameDirection;

    if (!IsValidWheel(wheel))
    {
        return;
    }

    targetCps = LimitI32(
        targetCps,
        -SPEED_PI_TARGET_LIMIT_CPS,
        SPEED_PI_TARGET_LIMIT_CPS
    );

    if (targetCps == 0)
    {
        SpeedPI_StopWheel(wheel);
        return;
    }

    controller = &g_controller[wheel];
    previousTarget = controller->status.targetCps;
    sameDirection = controller->status.enabled &&
        (((previousTarget > 0) && (targetCps > 0)) ||
         ((previousTarget < 0) && (targetCps < 0)));

    controller->status.enabled = true;
    controller->status.targetCps = targetCps;
    controller->status.errorCps =
        targetCps - controller->status.filteredCps;
    controller->status.saturated = false;

    if (!sameDirection)
    {
        controller->proportionalQ12 = 0;
        controller->integralQ12 = 0;
        controller->status.proportionalPercent = 0;
        controller->status.integralPercent = 0;
        controller->status.outputPercent = 0;
        controller->filterInitialized = false;
        Motor_Brake(MotorForWheel(wheel));
    }
}

void SpeedPI_SetTargets(int32_t motor1TargetCps, int32_t motor2TargetCps)
{
    SpeedPI_SetTarget(SPEED_PI_MOTOR1, motor1TargetCps);
    SpeedPI_SetTarget(SPEED_PI_MOTOR2, motor2TargetCps);
}

void SpeedPI_StopWheel(SpeedPIWheel_t wheel)
{
    SpeedPIController_t *controller;

    if (!IsValidWheel(wheel))
    {
        return;
    }

    controller = &g_controller[wheel];
    controller->status.enabled = false;
    controller->status.targetCps = 0;
    controller->status.errorCps = 0;
    controller->status.proportionalPercent = 0;
    controller->status.integralPercent = 0;
    controller->status.outputPercent = 0;
    controller->status.saturated = false;
    controller->proportionalQ12 = 0;
    controller->integralQ12 = 0;
    Motor_Brake(MotorForWheel(wheel));
}

void SpeedPI_StopAll(void)
{
    SpeedPI_StopWheel(SPEED_PI_MOTOR1);
    SpeedPI_StopWheel(SPEED_PI_MOTOR2);
}

void SpeedPI_ResetIntegral(SpeedPIWheel_t wheel)
{
    if (!IsValidWheel(wheel))
    {
        return;
    }

    g_controller[wheel].integralQ12 = 0;
    g_controller[wheel].status.integralPercent = 0;
    g_controller[wheel].status.saturated = false;
}

void SpeedPI_ResetAllIntegrals(void)
{
    SpeedPI_ResetIntegral(SPEED_PI_MOTOR1);
    SpeedPI_ResetIntegral(SPEED_PI_MOTOR2);
}


void SpeedPI_SetStartupMinimumPercent(
    SpeedPIWheel_t wheel,
    uint8_t percent)
{
    if (!IsValidWheel(wheel))
    {
        return;
    }

    if (percent > SPEED_PI_STARTUP_MIN_PERCENT_MAX)
    {
        percent = SPEED_PI_STARTUP_MIN_PERCENT_MAX;
    }

    g_controller[wheel].startupMinimumPercent = percent;
}

void SpeedPI_SetAllStartupMinimumPercent(uint8_t percent)
{
    SpeedPI_SetStartupMinimumPercent(SPEED_PI_MOTOR1, percent);
    SpeedPI_SetStartupMinimumPercent(SPEED_PI_MOTOR2, percent);
}

bool SpeedPI_SetGains(
    SpeedPIWheel_t wheel,
    int32_t kpQ12,
    int32_t kiQ12)
{
    if (!IsValidWheel(wheel) ||
        (kpQ12 < SPEED_PI_KP_Q12_MIN) ||
        (kpQ12 > SPEED_PI_KP_Q12_MAX) ||
        (kiQ12 < SPEED_PI_KI_Q12_MIN) ||
        (kiQ12 > SPEED_PI_KI_Q12_MAX))
    {
        return false;
    }

    g_gains[wheel].kpQ12 = kpQ12;
    g_gains[wheel].kiQ12 = kiQ12;
    g_controller[wheel].proportionalQ12 = 0;
    g_controller[wheel].integralQ12 = 0;
    g_controller[wheel].status.proportionalPercent = 0;
    g_controller[wheel].status.integralPercent = 0;
    g_controller[wheel].status.saturated = false;
    return true;
}

SpeedPIGains_t SpeedPI_GetGains(SpeedPIWheel_t wheel)
{
    SpeedPIGains_t invalid = {0, 0};
    return IsValidWheel(wheel) ? g_gains[wheel] : invalid;
}

SpeedPIWheelStatus_t SpeedPI_GetWheelStatus(SpeedPIWheel_t wheel)
{
    SpeedPIWheelStatus_t invalid = {0};
    return IsValidWheel(wheel) ? g_controller[wheel].status : invalid;
}

SpeedPIStatus_t SpeedPI_GetStatus(void)
{
    SpeedPIStatus_t status;
    status.motor1 = g_controller[SPEED_PI_MOTOR1].status;
    status.motor2 = g_controller[SPEED_PI_MOTOR2].status;
    return status;
}

void SpeedPI_PrintStatus(void)
{
    Serial_SendString("PI ");
    PrintWheel("M1", SPEED_PI_MOTOR1);
    Serial_SendString(" | ");
    PrintWheel("M2", SPEED_PI_MOTOR2);
    Serial_SendString("\r\n");
}

void SpeedPI_ControlTask(uint32_t nowMs)
{
    if (Encoder_Task(nowMs))
    {
        UpdateWheel(SPEED_PI_MOTOR1);
        UpdateWheel(SPEED_PI_MOTOR2);
    }
}
