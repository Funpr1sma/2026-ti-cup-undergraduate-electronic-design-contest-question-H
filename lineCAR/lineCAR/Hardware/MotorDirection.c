#include "MotorDirection.h"

#include "Serial.h"
#include "ti_msp_dl_config.h"

/*
 * 直接定义扩展板实际使用的GPIO。
 *
 * SysConfig仍然必须负责将这些引脚初始化为数字输出。
 */

/* TB6612通道A / 扩展板MOTOR2 */
#define MOTOR_AIN1_PORT       (GPIOB)
#define MOTOR_AIN1_PIN        (DL_GPIO_PIN_17)

#define MOTOR_AIN2_PORT       (GPIOB)
#define MOTOR_AIN2_PIN        (DL_GPIO_PIN_19)

/* TB6612通道B / 扩展板MOTOR1 */
#define MOTOR_BIN1_PORT       (GPIOA)
#define MOTOR_BIN1_PIN        (DL_GPIO_PIN_16)

#define MOTOR_BIN2_PORT       (GPIOB)
#define MOTOR_BIN2_PIN        (DL_GPIO_PIN_24)

/*
 * 模块保存的当前方向状态。
 */
static MotorDirectionState_t g_channelAState =
    MOTOR_DIRECTION_COAST;

static MotorDirectionState_t g_channelBState =
    MOTOR_DIRECTION_COAST;


/**
 * @brief 应用通道A方向状态。
 */
static void MotorDirection_ApplyChannelA(
    MotorDirectionState_t state)
{
    /*
     * 切换前先把AIN1、AIN2同时拉低，
     * 避免方向切换瞬间出现错误组合。
     */
    DL_GPIO_clearPins(
        MOTOR_AIN1_PORT,
        MOTOR_AIN1_PIN | MOTOR_AIN2_PIN
    );

    switch (state)
    {
        case MOTOR_DIRECTION_INPUT1_HIGH:
        {
            DL_GPIO_setPins(
                MOTOR_AIN1_PORT,
                MOTOR_AIN1_PIN
            );
            break;
        }

        case MOTOR_DIRECTION_INPUT2_HIGH:
        {
            DL_GPIO_setPins(
                MOTOR_AIN2_PORT,
                MOTOR_AIN2_PIN
            );
            break;
        }

        case MOTOR_DIRECTION_BRAKE:
        {
            DL_GPIO_setPins(
                MOTOR_AIN1_PORT,
                MOTOR_AIN1_PIN | MOTOR_AIN2_PIN
            );
            break;
        }

        case MOTOR_DIRECTION_COAST:
        default:
        {
            /*
             * 两个引脚已经被清零。
             */
            break;
        }
    }
}


/**
 * @brief 应用通道B方向状态。
 */
static void MotorDirection_ApplyChannelB(
    MotorDirectionState_t state)
{
    /*
     * BIN1位于GPIOA，BIN2位于GPIOB，
     * 因此需要分别操作两个端口。
     */
    DL_GPIO_clearPins(
        MOTOR_BIN1_PORT,
        MOTOR_BIN1_PIN
    );

    DL_GPIO_clearPins(
        MOTOR_BIN2_PORT,
        MOTOR_BIN2_PIN
    );

    switch (state)
    {
        case MOTOR_DIRECTION_INPUT1_HIGH:
        {
            DL_GPIO_setPins(
                MOTOR_BIN1_PORT,
                MOTOR_BIN1_PIN
            );
            break;
        }

        case MOTOR_DIRECTION_INPUT2_HIGH:
        {
            DL_GPIO_setPins(
                MOTOR_BIN2_PORT,
                MOTOR_BIN2_PIN
            );
            break;
        }

        case MOTOR_DIRECTION_BRAKE:
        {
            DL_GPIO_setPins(
                MOTOR_BIN1_PORT,
                MOTOR_BIN1_PIN
            );

            DL_GPIO_setPins(
                MOTOR_BIN2_PORT,
                MOTOR_BIN2_PIN
            );
            break;
        }

        case MOTOR_DIRECTION_COAST:
        default:
        {
            /*
             * 两个引脚已经被清零。
             */
            break;
        }
    }
}


/**
 * @brief 输出当前软件方向状态。
 */
static void MotorDirection_PrintState(void)
{
    Serial_SendString("STATE: ");

    switch (g_channelAState)
    {
        case MOTOR_DIRECTION_INPUT1_HIGH:
        {
            Serial_SendString("A=10");
            break;
        }

        case MOTOR_DIRECTION_INPUT2_HIGH:
        {
            Serial_SendString("A=01");
            break;
        }

        case MOTOR_DIRECTION_BRAKE:
        {
            Serial_SendString("A=11");
            break;
        }

        case MOTOR_DIRECTION_COAST:
        default:
        {
            Serial_SendString("A=00");
            break;
        }
    }

    Serial_SendString("  ");

    switch (g_channelBState)
    {
        case MOTOR_DIRECTION_INPUT1_HIGH:
        {
            Serial_SendString("B=10");
            break;
        }

        case MOTOR_DIRECTION_INPUT2_HIGH:
        {
            Serial_SendString("B=01");
            break;
        }

        case MOTOR_DIRECTION_BRAKE:
        {
            Serial_SendString("B=11");
            break;
        }

        case MOTOR_DIRECTION_COAST:
        default:
        {
            Serial_SendString("B=00");
            break;
        }
    }

    Serial_SendString("\r\n");
}


void MotorDirection_Init(void)
{
    /*
     * 保证四个方向输入全部为低电平。
     */
    MotorDirection_StopAll();

    Serial_SendString("\r\n");
    Serial_SendString(
        "Motor direction module initialized\r\n"
    );
    Serial_SendString(
        "Channel A / MOTOR2: "
        "AIN1=PB17 AIN2=PB19\r\n"
    );
    Serial_SendString(
        "Channel B / MOTOR1: "
        "BIN1=PA16 BIN2=PB24\r\n"
    );
}


void MotorDirection_SetChannelA(
    MotorDirectionState_t state)
{
    /*
     * 防止传入非法枚举值。
     */
    if (state > MOTOR_DIRECTION_BRAKE)
    {
        state = MOTOR_DIRECTION_COAST;
    }

    /*
     * 方向没有变化时禁止重复执行“先清零、再置位”。
     *
     * 速度PI每20 ms都会刷新一次电机输出。如果每次都重写
     * 方向GPIO，逻辑分析仪会在方向引脚上看到周期性窄脉冲，
     * 还会使H桥重复经历方向切换过程。
     */
    if (state == g_channelAState)
    {
        return;
    }

    MotorDirection_ApplyChannelA(state);
    g_channelAState = state;
}


void MotorDirection_SetChannelB(
    MotorDirectionState_t state)
{
    if (state > MOTOR_DIRECTION_BRAKE)
    {
        state = MOTOR_DIRECTION_COAST;
    }

    /* 与通道A相同：相同状态不重复翻转GPIO。 */
    if (state == g_channelBState)
    {
        return;
    }

    MotorDirection_ApplyChannelB(state);
    g_channelBState = state;
}


void MotorDirection_StopAll(void)
{
    MotorDirection_SetChannelA(
        MOTOR_DIRECTION_COAST
    );

    MotorDirection_SetChannelB(
        MOTOR_DIRECTION_COAST
    );
}


MotorDirectionState_t MotorDirection_GetChannelA(void)
{
    return g_channelAState;
}


MotorDirectionState_t MotorDirection_GetChannelB(void)
{
    return g_channelBState;
}


void MotorDirection_PrintTestHelp(void)
{
    Serial_SendString("\r\n");
    Serial_SendString(
        "Motor direction GPIO commands:\r\n"
    );

    Serial_SendString(
        "0: A=00, B=00, all direction pins low\r\n"
    );

    Serial_SendString(
        "1: A=10, B=00, PB17 high\r\n"
    );

    Serial_SendString(
        "2: A=01, B=00, PB19 high\r\n"
    );

    Serial_SendString(
        "3: A=00, B=10, PA16 high\r\n"
    );

    Serial_SendString(
        "4: A=00, B=01, PB24 high\r\n"
    );

    Serial_SendString(
        "5: A=10, B=10, PB17 and PA16 high\r\n"
    );

    Serial_SendString(
        "6: A=01, B=01, PB19 and PB24 high\r\n"
    );

    Serial_SendString(
        "p: print software state\r\n"
    );

    Serial_SendString(
        "h: print this help\r\n"
    );

    Serial_SendString(
        "Use text mode, no CR/LF append\r\n"
    );
}


void MotorDirection_TestTask(void)
{
    uint8_t command;

    if (!Serial_TryReadByte(&command))
    {
        return;
    }

    switch (command)
    {
        case (uint8_t)'0':
        {
            MotorDirection_StopAll();

            Serial_SendString(
                "Command 0: all direction pins low\r\n"
            );
            break;
        }

        case (uint8_t)'1':
        {
            MotorDirection_StopAll();

            MotorDirection_SetChannelA(
                MOTOR_DIRECTION_INPUT1_HIGH
            );

            Serial_SendString(
                "Command 1: PB17=1\r\n"
            );
            break;
        }

        case (uint8_t)'2':
        {
            MotorDirection_StopAll();

            MotorDirection_SetChannelA(
                MOTOR_DIRECTION_INPUT2_HIGH
            );

            Serial_SendString(
                "Command 2: PB19=1\r\n"
            );
            break;
        }

        case (uint8_t)'3':
        {
            MotorDirection_StopAll();

            MotorDirection_SetChannelB(
                MOTOR_DIRECTION_INPUT1_HIGH
            );

            Serial_SendString(
                "Command 3: PA16=1\r\n"
            );
            break;
        }

        case (uint8_t)'4':
        {
            MotorDirection_StopAll();

            MotorDirection_SetChannelB(
                MOTOR_DIRECTION_INPUT2_HIGH
            );

            Serial_SendString(
                "Command 4: PB24=1\r\n"
            );
            break;
        }

        case (uint8_t)'5':
        {
            MotorDirection_SetChannelA(
                MOTOR_DIRECTION_INPUT1_HIGH
            );

            MotorDirection_SetChannelB(
                MOTOR_DIRECTION_INPUT1_HIGH
            );

            Serial_SendString(
                "Command 5: PB17=1, PA16=1\r\n"
            );
            break;
        }

        case (uint8_t)'6':
        {
            MotorDirection_SetChannelA(
                MOTOR_DIRECTION_INPUT2_HIGH
            );

            MotorDirection_SetChannelB(
                MOTOR_DIRECTION_INPUT2_HIGH
            );

            Serial_SendString(
                "Command 6: PB19=1, PB24=1\r\n"
            );
            break;
        }

        case (uint8_t)'p':
        case (uint8_t)'P':
        {
            MotorDirection_PrintState();
            return;
        }

        case (uint8_t)'h':
        case (uint8_t)'H':
        {
            MotorDirection_PrintTestHelp();
            return;
        }

        case (uint8_t)'\r':
        case (uint8_t)'\n':
        {
            /*
             * 忽略串口助手附加的回车和换行。
             */
            return;
        }

        default:
        {
            Serial_SendString(
                "Unknown motor command\r\n"
            );
            return;
        }
    }

    MotorDirection_PrintState();
}