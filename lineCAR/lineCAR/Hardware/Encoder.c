#include "Encoder.h"

#include "Config/CarConfig.h"
#include "Serial.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * 速度采样周期。
 *
 * 后续速度PI也以20 ms为控制周期。
 */
#define ENCODER_SAMPLE_PERIOD_MS       APP_CONTROL_PERIOD_MS

/**
 * MOTOR1方向修正。
 *
 * 阶段8最终验收值为-1。
 */
#define ENCODER_M1_DIRECTION_SIGN      (-1)

/**
 * MOTOR2方向修正。
 *
 * 保持阶段7最终验收值。
 */
#define ENCODER_M2_DIRECTION_SIGN      (1)

/**
 * MOTOR1软件编码器GPIO。
 */
#define ENCODER_M1_PORT                (GPIOA)
#define ENCODER_M1_A_PIN               (DL_GPIO_PIN_25)
#define ENCODER_M1_B_PIN               (DL_GPIO_PIN_14)

#define ENCODER_M1_ALL_PINS            \
    (ENCODER_M1_A_PIN |                \
     ENCODER_M1_B_PIN)

typedef struct
{
    int32_t lastSourceCount;
    EncoderData_t data;

} EncoderInternalState_t;

/**
 * 正交状态转换表。
 *
 * 状态编码：
 * bit1 = A
 * bit0 = B
 *
 * 索引：
 * previousState << 2 | currentState
 */
static const int8_t g_quadratureTable[16] =
{
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

/**
 * 在GPIOA中断中更新的共享变量。
 */
static volatile int32_t g_motor1SoftwareCount = 0;
static volatile uint8_t g_motor1PreviousState = 0U;
static volatile uint32_t g_motor1InvalidTransitions = 0U;

/**
 * 0：MOTOR1
 * 1：MOTOR2
 */
static EncoderInternalState_t g_encoderState[2];

static uint32_t g_encoderLastSampleMs = 0U;
static bool g_encoderTimeInitialized = false;


static bool Encoder_IsValidId(EncoderId_t encoder)
{
    return (
        (encoder == ENCODER_ID_MOTOR1) ||
        (encoder == ENCODER_ID_MOTOR2)
    );
}


static uint8_t Encoder_ReadMotor1State(void)
{
    uint32_t gpioValue;
    uint8_t state = 0U;

    gpioValue = DL_GPIO_readPins(
        ENCODER_M1_PORT,
        ENCODER_M1_ALL_PINS
    );

    if ((gpioValue & ENCODER_M1_A_PIN) != 0U)
    {
        state |= 0x02U;
    }

    if ((gpioValue & ENCODER_M1_B_PIN) != 0U)
    {
        state |= 0x01U;
    }

    return state;
}


static uint16_t Encoder_ReadMotor2RawCount(void)
{
    return (uint16_t)
        DL_TimerG_getTimerCount(
            ENCODER_M2_INST
        );
}


static int32_t Encoder_GetDirectionSign(
    EncoderId_t encoder)
{
    if (encoder == ENCODER_ID_MOTOR1)
    {
        return ENCODER_M1_DIRECTION_SIGN;
    }

    return ENCODER_M2_DIRECTION_SIGN;
}


static void Encoder_SendUint32(uint32_t value)
{
    char buffer[10];
    uint32_t index = 0U;

    if (value == 0U)
    {
        Serial_SendByte((uint8_t)'0');
        return;
    }

    while ((value > 0U) &&
           (index < sizeof(buffer)))
    {
        buffer[index] =
            (char)('0' + (value % 10U));

        value /= 10U;
        index++;
    }

    while (index > 0U)
    {
        index--;

        Serial_SendByte(
            (uint8_t)buffer[index]
        );
    }
}


static void Encoder_SendInt32(
    int32_t value,
    bool showPositiveSign)
{
    uint32_t magnitude;

    if (value < 0)
    {
        Serial_SendByte((uint8_t)'-');

        magnitude =
            (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        if (showPositiveSign && (value > 0))
        {
            Serial_SendByte((uint8_t)'+');
        }

        magnitude = (uint32_t)value;
    }

    Encoder_SendUint32(magnitude);
}


static void Encoder_ClearData(
    EncoderData_t *data)
{
    if (data == 0)
    {
        return;
    }

    data->sourceCount = 0;
    data->deltaCount = 0;
    data->totalCount = 0;
    data->cps = 0;
    data->sampleIntervalMs = 0U;
    data->invalidTransitions = 0U;
}


static void Encoder_UpdateMotor1(uint32_t elapsedMs)
{
    int32_t currentSourceCount;
    int32_t rawDifference;
    int32_t correctedDifference;

    EncoderInternalState_t *state;

    if (elapsedMs == 0U)
    {
        return;
    }

    state =
        &g_encoderState[ENCODER_ID_MOTOR1];

    /**
     * 32位对齐读取在Cortex-M0+上是单次访问。
     */
    currentSourceCount =
        g_motor1SoftwareCount;

    rawDifference =
        currentSourceCount -
        state->lastSourceCount;

    correctedDifference =
        rawDifference *
        Encoder_GetDirectionSign(
            ENCODER_ID_MOTOR1
        );

    state->data.sourceCount =
        currentSourceCount;

    state->data.deltaCount =
        correctedDifference;

    state->data.totalCount +=
        correctedDifference;

    state->data.cps =
        (correctedDifference * 1000) /
        (int32_t)elapsedMs;

    state->data.sampleIntervalMs =
        elapsedMs;

    state->data.invalidTransitions =
        g_motor1InvalidTransitions;

    state->lastSourceCount =
        currentSourceCount;
}


static void Encoder_UpdateMotor2(uint32_t elapsedMs)
{
    uint16_t currentRawCount;
    uint16_t previousRawCount;
    uint16_t unsignedDifference;
    int16_t signedDifference;

    int32_t correctedDifference;

    EncoderInternalState_t *state;

    if (elapsedMs == 0U)
    {
        return;
    }

    state =
        &g_encoderState[ENCODER_ID_MOTOR2];

    currentRawCount =
        Encoder_ReadMotor2RawCount();

    previousRawCount =
        (uint16_t)state->lastSourceCount;

    /**
     * 16位无符号减法自动处理回绕。
     */
    unsignedDifference =
        (uint16_t)(
            currentRawCount -
            previousRawCount
        );

    signedDifference =
        (int16_t)unsignedDifference;

    correctedDifference =
        (int32_t)signedDifference *
        Encoder_GetDirectionSign(
            ENCODER_ID_MOTOR2
        );

    state->data.sourceCount =
        (int32_t)currentRawCount;

    state->data.deltaCount =
        correctedDifference;

    state->data.totalCount +=
        correctedDifference;

    state->data.cps =
        (correctedDifference * 1000) /
        (int32_t)elapsedMs;

    state->data.sampleIntervalMs =
        elapsedMs;

    state->data.invalidTransitions = 0U;

    state->lastSourceCount =
        (int32_t)currentRawCount;
}


void Encoder_Init(void)
{
    uint16_t motor2RawCount;

    /**
     * 初始化MOTOR1软件正交解码。
     */
    NVIC_DisableIRQ(GPIOA_INT_IRQn);

    g_motor1SoftwareCount = 0;
    g_motor1InvalidTransitions = 0U;

    g_motor1PreviousState =
        Encoder_ReadMotor1State();

    DL_GPIO_clearInterruptStatus(
        ENCODER_M1_PORT,
        ENCODER_M1_ALL_PINS
    );

    DL_GPIO_enableInterrupt(
        ENCODER_M1_PORT,
        ENCODER_M1_ALL_PINS
    );

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /**
     * 启动MOTOR2硬件QEI。
     */
    DL_TimerG_startCounter(
        ENCODER_M2_INST
    );

    motor2RawCount =
        Encoder_ReadMotor2RawCount();

    Encoder_ClearData(
        &g_encoderState[
            ENCODER_ID_MOTOR1
        ].data
    );

    Encoder_ClearData(
        &g_encoderState[
            ENCODER_ID_MOTOR2
        ].data
    );

    g_encoderState[
        ENCODER_ID_MOTOR1
    ].lastSourceCount = 0;

    g_encoderState[
        ENCODER_ID_MOTOR2
    ].lastSourceCount =
        (int32_t)motor2RawCount;

    g_encoderState[
        ENCODER_ID_MOTOR2
    ].data.sourceCount =
        (int32_t)motor2RawCount;

    g_encoderLastSampleMs = 0U;
    g_encoderTimeInitialized = false;

    Serial_SendString("\r\n");
    Serial_SendString(
        "Dual encoder initialized\r\n"
    );
    Serial_SendString(
        "M1: PA25/PA14 Software QEI, sign=-1\r\n"
    );
    Serial_SendString(
        "M2: PA26/PA27 TIMG8 QEI\r\n"
    );
    Serial_SendString(
        "Sample period: 20 ms\r\n"
    );
}


void Encoder_ResetAll(void)
{
    uint32_t interruptState;
    uint16_t motor2RawCount;

    interruptState = __get_PRIMASK();

    __disable_irq();

    g_motor1SoftwareCount = 0;
    g_motor1InvalidTransitions = 0U;

    g_motor1PreviousState =
        Encoder_ReadMotor1State();

    DL_GPIO_clearInterruptStatus(
        ENCODER_M1_PORT,
        ENCODER_M1_ALL_PINS
    );

    if (interruptState == 0U)
    {
        __enable_irq();
    }

    motor2RawCount =
        Encoder_ReadMotor2RawCount();

    Encoder_ClearData(
        &g_encoderState[
            ENCODER_ID_MOTOR1
        ].data
    );

    Encoder_ClearData(
        &g_encoderState[
            ENCODER_ID_MOTOR2
        ].data
    );

    g_encoderState[
        ENCODER_ID_MOTOR1
    ].lastSourceCount = 0;

    g_encoderState[
        ENCODER_ID_MOTOR2
    ].lastSourceCount =
        (int32_t)motor2RawCount;

    g_encoderState[
        ENCODER_ID_MOTOR2
    ].data.sourceCount =
        (int32_t)motor2RawCount;

    g_encoderTimeInitialized = false;
}


void Encoder_Update(uint32_t elapsedMs)
{
    Encoder_UpdateMotor1(elapsedMs);
    Encoder_UpdateMotor2(elapsedMs);
}


bool Encoder_Task(uint32_t nowMs)
{
    uint32_t elapsedMs;

    if (!g_encoderTimeInitialized)
    {
        g_encoderLastSampleMs = nowMs;
        g_encoderTimeInitialized = true;

        return false;
    }

    elapsedMs =
        (uint32_t)(
            nowMs - g_encoderLastSampleMs
        );

    if (elapsedMs <
        ENCODER_SAMPLE_PERIOD_MS)
    {
        return false;
    }

    /**
     * 使用实际经过时间计算CPS。
     */
    g_encoderLastSampleMs = nowMs;

    Encoder_Update(elapsedMs);

    return true;
}


EncoderData_t Encoder_GetData(
    EncoderId_t encoder)
{
    EncoderData_t invalidData;

    if (Encoder_IsValidId(encoder))
    {
        return g_encoderState[encoder].data;
    }

    Encoder_ClearData(&invalidData);

    return invalidData;
}


int32_t Encoder_GetMotor1Cps(void)
{
    return g_encoderState[
        ENCODER_ID_MOTOR1
    ].data.cps;
}


int32_t Encoder_GetMotor2Cps(void)
{
    return g_encoderState[
        ENCODER_ID_MOTOR2
    ].data.cps;
}


int32_t Encoder_GetMotor1SoftwareCount(void)
{
    return g_motor1SoftwareCount;
}


void Encoder_PrintStatus(void)
{
    const EncoderData_t *motor1;
    const EncoderData_t *motor2;

    motor1 =
        &g_encoderState[
            ENCODER_ID_MOTOR1
        ].data;

    motor2 =
        &g_encoderState[
            ENCODER_ID_MOTOR2
        ].data;

    Serial_SendString("ENC: M1 cps=");

    Encoder_SendInt32(
        motor1->cps,
        true
    );

    Serial_SendString(" d=");

    Encoder_SendInt32(
        motor1->deltaCount,
        true
    );

    Serial_SendString(" total=");

    Encoder_SendInt32(
        motor1->totalCount,
        false
    );

    Serial_SendString(" bad=");

    Encoder_SendUint32(
        motor1->invalidTransitions
    );

    Serial_SendString(" | M2 cps=");

    Encoder_SendInt32(
        motor2->cps,
        true
    );

    Serial_SendString(" d=");

    Encoder_SendInt32(
        motor2->deltaCount,
        true
    );

    Serial_SendString("\r\n");
}


/**
 * GPIOA端口中断服务函数。
 *
 * 项目中只能存在一个GROUP1_IRQHandler。
 */
void GROUP1_IRQHandler(void)
{
    uint32_t pendingPins;
    uint8_t currentState;
    uint8_t tableIndex;
    int8_t step;

    pendingPins =
        DL_GPIO_getEnabledInterruptStatus(
            ENCODER_M1_PORT,
            ENCODER_M1_ALL_PINS
        );

    if (pendingPins == 0U)
    {
        return;
    }

    currentState =
        Encoder_ReadMotor1State();

    tableIndex =
        (uint8_t)(
            (g_motor1PreviousState << 2) |
            currentState
        );

    step =
        g_quadratureTable[tableIndex];

    if (step != 0)
    {
        g_motor1SoftwareCount +=
            (int32_t)step;
    }
    else if (currentState !=
             g_motor1PreviousState)
    {
        g_motor1InvalidTransitions++;
    }

    g_motor1PreviousState =
        currentState;

    DL_GPIO_clearInterruptStatus(
        ENCODER_M1_PORT,
        pendingPins
    );
}