#include "PWM.h"

#include "Serial.h"
#include "ti_msp_dl_config.h"

/*
 * SysConfig中必须设置相同的Timer Count。
 *
 * 32 MHz / 1600 = 20 kHz。
 */
#define MOTOR_PWM_PERIOD_COUNT    (1600U)

#define MOTOR_PWM_DUTY_MIN        (0U)
#define MOTOR_PWM_DUTY_MAX        (100U)

/*
 * 软件记录的当前占空比。
 */
static uint8_t g_channelADuty = 0U;
static uint8_t g_channelBDuty = 0U;


/**
 * @brief 限制占空比范围。
 */
static uint8_t PWM_LimitDuty(uint8_t dutyPercent)
{
    if (dutyPercent > MOTOR_PWM_DUTY_MAX)
    {
        return MOTOR_PWM_DUTY_MAX;
    }

    return dutyPercent;
}


/**
 * @brief 将百分比占空比换算为定时器比较值。
 *
 * 例如：
 *
 * 0%   -> 0
 * 25%  -> 400
 * 50%  -> 800
 * 75%  -> 1200
 * 100% -> 1600
 */
static uint32_t PWM_DutyToCompare(
    uint8_t dutyPercent)
{
    uint32_t compareValue;

    dutyPercent = PWM_LimitDuty(dutyPercent);

    compareValue =
        ((uint32_t)dutyPercent *
         MOTOR_PWM_PERIOD_COUNT) /
        100U;

    return compareValue;
}


/**
 * @brief 发送一个无符号十进制整数。
 */
static void PWM_SendUint32(uint32_t value)
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


/**
 * @brief 输出当前PWM状态。
 */
static void PWM_PrintStatus(void)
{
    Serial_SendString("PWM: A=");

    PWM_SendUint32(
        (uint32_t)g_channelADuty
    );

    Serial_SendString("%  B=");

    PWM_SendUint32(
        (uint32_t)g_channelBDuty
    );

    Serial_SendString("%\r\n");
}


void PWM_Init(void)
{
    /*
     * SysConfig已经完成：
     *
     * TIMG0时钟配置；
     * Edge-aligned PWM模式；
     * PA12和PA13引脚复用；
     * CC0和CC1输出配置。
     */

    g_channelADuty = 0U;
    g_channelBDuty = 0U;

    /*
     * 启动前保证两路比较值均为0。
     */
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST,
        0U,
        DL_TIMER_CC_0_INDEX
    );

    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST,
        0U,
        DL_TIMER_CC_1_INDEX
    );

    /*
     * 启动TIMG0。
     */
    DL_TimerG_startCounter(MOTOR_PWM_INST);

    Serial_SendString("\r\n");
    Serial_SendString(
        "Motor PWM module initialized\r\n"
    );

    Serial_SendString(
        "Channel A / MOTOR2: "
        "PA12 TIMG0_C0\r\n"
    );

    Serial_SendString(
        "Channel B / MOTOR1: "
        "PA13 TIMG0_C1\r\n"
    );

    Serial_SendString(
        "PWM frequency: 20 kHz\r\n"
    );

    PWM_PrintStatus();
}


void PWM_SetDuty(
    PWMChannel_t channel,
    uint8_t dutyPercent)
{
    uint32_t compareValue;

    dutyPercent = PWM_LimitDuty(dutyPercent);

    compareValue =
        PWM_DutyToCompare(dutyPercent);

    switch (channel)
    {
        case PWM_CHANNEL_A:
        {
            DL_TimerG_setCaptureCompareValue(
                MOTOR_PWM_INST,
                compareValue,
                DL_TIMER_CC_0_INDEX
            );

            g_channelADuty = dutyPercent;
            break;
        }

        case PWM_CHANNEL_B:
        {
            DL_TimerG_setCaptureCompareValue(
                MOTOR_PWM_INST,
                compareValue,
                DL_TIMER_CC_1_INDEX
            );

            g_channelBDuty = dutyPercent;
            break;
        }

        default:
        {
            /*
             * 非法通道不做处理。
             */
            break;
        }
    }
}


void PWM_SetBothDuty(
    uint8_t channelADuty,
    uint8_t channelBDuty)
{
    PWM_SetDuty(
        PWM_CHANNEL_A,
        channelADuty
    );

    PWM_SetDuty(
        PWM_CHANNEL_B,
        channelBDuty
    );
}


void PWM_StopAll(void)
{
    PWM_SetBothDuty(
        MOTOR_PWM_DUTY_MIN,
        MOTOR_PWM_DUTY_MIN
    );
}


uint8_t PWM_GetChannelADuty(void)
{
    return g_channelADuty;
}


uint8_t PWM_GetChannelBDuty(void)
{
    return g_channelBDuty;
}


void PWM_PrintTestHelp(void)
{
    Serial_SendString("\r\n");
    Serial_SendString(
        "Dual PWM test commands:\r\n"
    );

    Serial_SendString(
        "0: A=0%,  B=0%\r\n"
    );

    Serial_SendString(
        "1: A=25%, B=0%\r\n"
    );

    Serial_SendString(
        "2: A=50%, B=0%\r\n"
    );

    Serial_SendString(
        "3: A=75%, B=0%\r\n"
    );

    Serial_SendString(
        "4: A=0%,  B=25%\r\n"
    );

    Serial_SendString(
        "5: A=0%,  B=50%\r\n"
    );

    Serial_SendString(
        "6: A=0%,  B=75%\r\n"
    );

    Serial_SendString(
        "7: A=25%, B=25%\r\n"
    );

    Serial_SendString(
        "8: A=50%, B=50%\r\n"
    );

    Serial_SendString(
        "9: A=75%, B=75%\r\n"
    );

    Serial_SendString(
        "p: print PWM status\r\n"
    );

    Serial_SendString(
        "h: print this help\r\n"
    );

    Serial_SendString(
        "Text mode, no CR/LF append\r\n"
    );
}


void PWM_TestTask(void)
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
            PWM_SetBothDuty(0U, 0U);
            break;
        }

        case (uint8_t)'1':
        {
            PWM_SetBothDuty(25U, 0U);
            break;
        }

        case (uint8_t)'2':
        {
            PWM_SetBothDuty(50U, 0U);
            break;
        }

        case (uint8_t)'3':
        {
            PWM_SetBothDuty(75U, 0U);
            break;
        }

        case (uint8_t)'4':
        {
            PWM_SetBothDuty(0U, 25U);
            break;
        }

        case (uint8_t)'5':
        {
            PWM_SetBothDuty(0U, 50U);
            break;
        }

        case (uint8_t)'6':
        {
            PWM_SetBothDuty(0U, 75U);
            break;
        }

        case (uint8_t)'7':
        {
            PWM_SetBothDuty(25U, 25U);
            break;
        }

        case (uint8_t)'8':
        {
            PWM_SetBothDuty(50U, 50U);
            break;
        }

        case (uint8_t)'9':
        {
            PWM_SetBothDuty(75U, 75U);
            break;
        }

        case (uint8_t)'p':
        case (uint8_t)'P':
        {
            PWM_PrintStatus();
            return;
        }

        case (uint8_t)'h':
        case (uint8_t)'H':
        {
            PWM_PrintTestHelp();
            return;
        }

        case (uint8_t)'\r':
        case (uint8_t)'\n':
        {
            /*
             * 忽略附加的回车换行。
             */
            return;
        }

        default:
        {
            Serial_SendString(
                "Unknown PWM command\r\n"
            );
            return;
        }
    }

    PWM_PrintStatus();
}