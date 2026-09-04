#include "Motor.h"

#include "MotorDirection.h"
#include "PWM.h"
#include "Serial.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * 阶段6测试功率。
 *
 * 第一次让电机实际转动时不要设置得太高。
 */
#define MOTOR_TEST_DUTY_PERCENT       (30U)

/**
 * 每条运动命令最长运行时间。
 */
#define MOTOR_TEST_TIMEOUT_MS         (5000U)

/**
 * 电机允许的最大占空比。
 */
#define MOTOR_MAX_DUTY_PERCENT        (100U)

/**
 * MOTOR1正转时使用的方向组合。
 *
 * MOTOR1对应TB6612通道B：
 * BIN1=PA16
 * BIN2=PB24
 *
 * 如果实测方向与小车前进方向相反，
 * 将INPUT1_HIGH改成INPUT2_HIGH。
 */
#define MOTOR1_FORWARD_DIRECTION      \
    (MOTOR_DIRECTION_INPUT1_HIGH)

/**
 * MOTOR2正转时使用的方向组合。
 *
 * MOTOR2对应TB6612通道A：
 * AIN1=PB17
 * AIN2=PB19
 *
 * 如果实测方向与小车前进方向相反，
 * 将INPUT1_HIGH改成INPUT2_HIGH。
 */
#define MOTOR2_FORWARD_DIRECTION      \
    (MOTOR_DIRECTION_INPUT1_HIGH)

/**
 * 两个电机的软件状态。
 *
 * 下标0对应MOTOR1；
 * 下标1对应MOTOR2。
 */
static MotorStatus_t g_motorStatus[2];

/**
 * 阶段6测试命令是否正在运行。
 */
static bool g_motorTestRunning = false;

/**
 * 当前运动测试开始时间。
 */
static uint32_t g_motorTestStartMs = 0U;


/**
 * @brief 判断电机编号是否合法。
 */
static bool Motor_IsValidId(MotorId_t motor)
{
    return (
        (motor == MOTOR_ID_1) ||
        (motor == MOTOR_ID_2)
    );
}


/**
 * @brief 限制占空比到0～100。
 */
static uint8_t Motor_LimitDuty(uint8_t dutyPercent)
{
    if (dutyPercent > MOTOR_MAX_DUTY_PERCENT)
    {
        return MOTOR_MAX_DUTY_PERCENT;
    }

    return dutyPercent;
}


/**
 * @brief 根据电机编号取得PWM底层通道。
 *
 * 扩展板映射：
 *
 * MOTOR1 -> TB6612通道B -> PWM_CHANNEL_B
 * MOTOR2 -> TB6612通道A -> PWM_CHANNEL_A
 */
static PWMChannel_t Motor_GetPwmChannel(
    MotorId_t motor)
{
    if (motor == MOTOR_ID_1)
    {
        return PWM_CHANNEL_B;
    }

    return PWM_CHANNEL_A;
}


/**
 * @brief 设置指定电机的方向GPIO。
 */
static void Motor_SetDirection(
    MotorId_t motor,
    MotorDirectionState_t direction)
{
    if (motor == MOTOR_ID_1)
    {
        /*
         * MOTOR1对应TB6612通道B。
         */
        MotorDirection_SetChannelB(direction);
    }
    else
    {
        /*
         * MOTOR2对应TB6612通道A。
         */
        MotorDirection_SetChannelA(direction);
    }
}


/**
 * @brief 取得指定电机的正转方向组合。
 */
static MotorDirectionState_t Motor_GetForwardDirection(
    MotorId_t motor)
{
    if (motor == MOTOR_ID_1)
    {
        return MOTOR1_FORWARD_DIRECTION;
    }

    return MOTOR2_FORWARD_DIRECTION;
}


/**
 * @brief 取得指定电机的反转方向组合。
 */
static MotorDirectionState_t Motor_GetReverseDirection(
    MotorId_t motor)
{
    MotorDirectionState_t forwardDirection;

    forwardDirection =
        Motor_GetForwardDirection(motor);

    if (forwardDirection ==
        MOTOR_DIRECTION_INPUT1_HIGH)
    {
        return MOTOR_DIRECTION_INPUT2_HIGH;
    }

    return MOTOR_DIRECTION_INPUT1_HIGH;
}


/**
 * @brief 设置指定电机PWM。
 */
static void Motor_SetPwm(
    MotorId_t motor,
    uint8_t dutyPercent)
{
    PWM_SetDuty(
        Motor_GetPwmChannel(motor),
        dutyPercent
    );
}


/**
 * @brief 向串口发送无符号十进制整数。
 */
static void Motor_SendUint32(uint32_t value)
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
 * @brief 输出电机工作模式名称。
 */
static void Motor_SendModeName(MotorMode_t mode)
{
    switch (mode)
    {
        case MOTOR_MODE_FORWARD:
        {
            Serial_SendString("FORWARD");
            break;
        }

        case MOTOR_MODE_REVERSE:
        {
            Serial_SendString("REVERSE");
            break;
        }

        case MOTOR_MODE_BRAKE:
        {
            Serial_SendString("BRAKE");
            break;
        }

        case MOTOR_MODE_COAST:
        default:
        {
            Serial_SendString("COAST");
            break;
        }
    }
}


/**
 * @brief 输出两个电机当前状态。
 */
static void Motor_PrintStatus(void)
{
    Serial_SendString("MOTOR1: ");
    Motor_SendModeName(
        g_motorStatus[MOTOR_ID_1].mode
    );

    Serial_SendString(" ");

    Motor_SendUint32(
        (uint32_t)
        g_motorStatus[MOTOR_ID_1].dutyPercent
    );

    Serial_SendString("%");

    Serial_SendString("  |  MOTOR2: ");

    Motor_SendModeName(
        g_motorStatus[MOTOR_ID_2].mode
    );

    Serial_SendString(" ");

    Motor_SendUint32(
        (uint32_t)
        g_motorStatus[MOTOR_ID_2].dutyPercent
    );

    Serial_SendString("%\r\n");
}


/**
 * @brief 启动阶段6的自动停止计时。
 */
static void Motor_StartTestTimeout(uint32_t nowMs)
{
    g_motorTestStartMs = nowMs;
    g_motorTestRunning = true;
}


/**
 * @brief 停止阶段6的自动停止计时。
 */
static void Motor_CancelTestTimeout(void)
{
    g_motorTestRunning = false;
}


/**
 * @brief 检查阶段6测试是否超时。
 */
static void Motor_CheckTestTimeout(uint32_t nowMs)
{
    if (!g_motorTestRunning)
    {
        return;
    }

    if ((uint32_t)(nowMs - g_motorTestStartMs) >=
        MOTOR_TEST_TIMEOUT_MS)
    {
        Motor_BrakeAll();
        g_motorTestRunning = false;

        Serial_SendString(
            "Motor test timeout: BRAKE\r\n"
        );

        Motor_PrintStatus();
    }
}


void Motor_Init(void)
{
    /*
     * 先将方向引脚全部初始化为低电平。
     */
    MotorDirection_Init();

    /*
     * 初始化并启动TIMG0双路PWM。
     * 初始化后两路占空比均为0%。
     */
    PWM_Init();

    /*
     * 先标记为COAST，随后Motor_BrakeAll()会执行一次真正的
     * COAST -> BRAKE状态转换并配置方向GPIO。
     */
    g_motorStatus[MOTOR_ID_1].mode =
        MOTOR_MODE_COAST;

    g_motorStatus[MOTOR_ID_1].dutyPercent = 0U;
    g_motorStatus[MOTOR_ID_1].signedCommand = 0;

    g_motorStatus[MOTOR_ID_2].mode =
        MOTOR_MODE_COAST;

    g_motorStatus[MOTOR_ID_2].dutyPercent = 0U;
    g_motorStatus[MOTOR_ID_2].signedCommand = 0;

    g_motorTestRunning = false;
    g_motorTestStartMs = 0U;

    /*
     * 初始化完成后进入安全制动状态。
     */
    Motor_BrakeAll();

    Serial_SendString("\r\n");
    Serial_SendString(
        "Motor module initialized\r\n"
    );

    Serial_SendString(
        "MOTOR1: PWM=PA13, BIN1=PA16, "
        "BIN2=PB24\r\n"
    );

    Serial_SendString(
        "MOTOR2: PWM=PA12, AIN1=PB17, "
        "AIN2=PB19\r\n"
    );

    Serial_SendString(
        "Test duty: 30%, timeout: 1500 ms\r\n"
    );

    Motor_PrintStatus();
}


void Motor_Forward(
    MotorId_t motor,
    uint8_t dutyPercent)
{
    MotorDirectionState_t direction;

    if (!Motor_IsValidId(motor))
    {
        return;
    }

    dutyPercent = Motor_LimitDuty(dutyPercent);

    if (dutyPercent == 0U)
    {
        /*
         * 0%只关闭PWM，不改方向引脚。
         * 显式停车仍由Motor_Brake()完成。
         */
        Motor_SetPwm(motor, 0U);
        g_motorStatus[motor].dutyPercent = 0U;
        g_motorStatus[motor].signedCommand = 0;
        return;
    }

    /*
     * 只有模式真正发生变化时，才执行PWM消隐和方向切换。
     * 正向运行期间的普通占空比更新只改比较值，避免每20 ms
     * 把PWM清零一次并在方向GPIO上制造窄脉冲。
     */
    if (g_motorStatus[motor].mode !=
        MOTOR_MODE_FORWARD)
    {
        Motor_SetPwm(motor, 0U);

        direction =
            Motor_GetForwardDirection(motor);

        Motor_SetDirection(motor, direction);

        g_motorStatus[motor].mode =
            MOTOR_MODE_FORWARD;
    }

    Motor_SetPwm(motor, dutyPercent);

    g_motorStatus[motor].dutyPercent =
        dutyPercent;

    g_motorStatus[motor].signedCommand =
        (int16_t)dutyPercent;
}


void Motor_Reverse(
    MotorId_t motor,
    uint8_t dutyPercent)
{
    MotorDirectionState_t direction;

    if (!Motor_IsValidId(motor))
    {
        return;
    }

    dutyPercent = Motor_LimitDuty(dutyPercent);

    if (dutyPercent == 0U)
    {
        Motor_SetPwm(motor, 0U);
        g_motorStatus[motor].dutyPercent = 0U;
        g_motorStatus[motor].signedCommand = 0;
        return;
    }

    /* 只有正反方向真正改变时才执行安全消隐。 */
    if (g_motorStatus[motor].mode !=
        MOTOR_MODE_REVERSE)
    {
        Motor_SetPwm(motor, 0U);

        direction =
            Motor_GetReverseDirection(motor);

        Motor_SetDirection(motor, direction);

        g_motorStatus[motor].mode =
            MOTOR_MODE_REVERSE;
    }

    Motor_SetPwm(motor, dutyPercent);

    g_motorStatus[motor].dutyPercent =
        dutyPercent;

    g_motorStatus[motor].signedCommand =
        -(int16_t)dutyPercent;
}


void Motor_SetSpeed(
    MotorId_t motor,
    int16_t speedPercent)
{
    int32_t extendedSpeed;

    if (!Motor_IsValidId(motor))
    {
        return;
    }

    extendedSpeed = (int32_t)speedPercent;

    if (extendedSpeed > 100)
    {
        extendedSpeed = 100;
    }
    else if (extendedSpeed < -100)
    {
        extendedSpeed = -100;
    }

    if (extendedSpeed > 0)
    {
        Motor_Forward(
            motor,
            (uint8_t)extendedSpeed
        );
    }
    else if (extendedSpeed < 0)
    {
        Motor_Reverse(
            motor,
            (uint8_t)(-extendedSpeed)
        );
    }
    else
    {
        /*
         * PI输出在目标附近可能因整数取整短暂变成0。
         * 这里仅把PWM置0并保持当前方向，不把0当作新的BRAKE
         * 状态，否则会在FORWARD与BRAKE之间以控制频率来回切换。
         *
         * 真正停车由SpeedPI_StopWheel()显式调用Motor_Brake()。
         */
        Motor_SetPwm(motor, 0U);

        g_motorStatus[motor].dutyPercent = 0U;
        g_motorStatus[motor].signedCommand = 0;
    }
}


void Motor_Brake(MotorId_t motor)
{
    if (!Motor_IsValidId(motor))
    {
        return;
    }

    Motor_SetPwm(motor, 0U);

    /* 已经是BRAKE时不重复翻转方向GPIO。 */
    if (g_motorStatus[motor].mode !=
        MOTOR_MODE_BRAKE)
    {
        Motor_SetDirection(
            motor,
            MOTOR_DIRECTION_BRAKE
        );

        g_motorStatus[motor].mode =
            MOTOR_MODE_BRAKE;
    }

    g_motorStatus[motor].dutyPercent = 0U;
    g_motorStatus[motor].signedCommand = 0;
}


void Motor_Coast(MotorId_t motor)
{
    if (!Motor_IsValidId(motor))
    {
        return;
    }

    Motor_SetPwm(motor, 0U);

    /* 已经是COAST时不重复翻转方向GPIO。 */
    if (g_motorStatus[motor].mode !=
        MOTOR_MODE_COAST)
    {
        Motor_SetDirection(
            motor,
            MOTOR_DIRECTION_COAST
        );

        g_motorStatus[motor].mode =
            MOTOR_MODE_COAST;
    }

    g_motorStatus[motor].dutyPercent = 0U;
    g_motorStatus[motor].signedCommand = 0;
}


void Motor_BrakeAll(void)
{
    Motor_Brake(MOTOR_ID_1);
    Motor_Brake(MOTOR_ID_2);
}


void Motor_CoastAll(void)
{
    Motor_Coast(MOTOR_ID_1);
    Motor_Coast(MOTOR_ID_2);
}


MotorStatus_t Motor_GetStatus(MotorId_t motor)
{
    MotorStatus_t invalidStatus;

    if (Motor_IsValidId(motor))
    {
        return g_motorStatus[motor];
    }

    invalidStatus.mode = MOTOR_MODE_BRAKE;
    invalidStatus.dutyPercent = 0U;
    invalidStatus.signedCommand = 0;

    return invalidStatus;
}


void Motor_PrintTestHelp(void)
{
    Serial_SendString("\r\n");
    Serial_SendString(
        "Stage 6 motor test commands:\r\n"
    );

    Serial_SendString(
        "0: brake MOTOR1 and MOTOR2\r\n"
    );

    Serial_SendString(
        "c: coast MOTOR1 and MOTOR2\r\n"
    );

    Serial_SendString(
        "1: MOTOR1 forward 30%\r\n"
    );

    Serial_SendString(
        "2: MOTOR1 reverse 30%\r\n"
    );

    Serial_SendString(
        "3: MOTOR2 forward 30%\r\n"
    );

    Serial_SendString(
        "4: MOTOR2 reverse 30%\r\n"
    );

    Serial_SendString(
        "5: both motors forward 30%\r\n"
    );

    Serial_SendString(
        "6: both motors reverse 30%\r\n"
    );

    Serial_SendString(
        "7: MOTOR1 forward, MOTOR2 reverse\r\n"
    );

    Serial_SendString(
        "8: MOTOR1 reverse, MOTOR2 forward\r\n"
    );

    Serial_SendString(
        "p: print motor status\r\n"
    );

    Serial_SendString(
        "h: print this help\r\n"
    );

    Serial_SendString(
        "Motion commands stop automatically "
        "after 1500 ms\r\n"
    );

    Serial_SendString(
        "Text mode, no CR/LF append\r\n"
    );
}


void Motor_TestTask(uint32_t nowMs)
{
    uint8_t command;
    bool motionCommand = false;

    /*
     * 即使没有串口数据，也必须持续检查超时。
     */
    Motor_CheckTestTimeout(nowMs);

    if (!Serial_TryReadByte(&command))
    {
        return;
    }

    switch (command)
    {
        case (uint8_t)'0':
        {
            Motor_BrakeAll();
            Motor_CancelTestTimeout();

            Serial_SendString(
                "Command 0: both motors BRAKE\r\n"
            );
            break;
        }

        case (uint8_t)'c':
        case (uint8_t)'C':
        {
            Motor_CoastAll();
            Motor_CancelTestTimeout();

            Serial_SendString(
                "Command c: both motors COAST\r\n"
            );
            break;
        }

        case (uint8_t)'1':
        {
            Motor_BrakeAll();

            Motor_Forward(
                MOTOR_ID_1,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 1: MOTOR1 FORWARD\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'2':
        {
            Motor_BrakeAll();

            Motor_Reverse(
                MOTOR_ID_1,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 2: MOTOR1 REVERSE\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'3':
        {
            Motor_BrakeAll();

            Motor_Forward(
                MOTOR_ID_2,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 3: MOTOR2 FORWARD\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'4':
        {
            Motor_BrakeAll();

            Motor_Reverse(
                MOTOR_ID_2,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 4: MOTOR2 REVERSE\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'5':
        {
            Motor_Forward(
                MOTOR_ID_1,
                MOTOR_TEST_DUTY_PERCENT
            );

            Motor_Forward(
                MOTOR_ID_2,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 5: both FORWARD\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'6':
        {
            Motor_Reverse(
                MOTOR_ID_1,
                MOTOR_TEST_DUTY_PERCENT
            );

            Motor_Reverse(
                MOTOR_ID_2,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 6: both REVERSE\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'7':
        {
            Motor_Forward(
                MOTOR_ID_1,
                MOTOR_TEST_DUTY_PERCENT
            );

            Motor_Reverse(
                MOTOR_ID_2,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 7: M1 FORWARD, "
                "M2 REVERSE\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'8':
        {
            Motor_Reverse(
                MOTOR_ID_1,
                MOTOR_TEST_DUTY_PERCENT
            );

            Motor_Forward(
                MOTOR_ID_2,
                MOTOR_TEST_DUTY_PERCENT
            );

            Serial_SendString(
                "Command 8: M1 REVERSE, "
                "M2 FORWARD\r\n"
            );

            motionCommand = true;
            break;
        }

        case (uint8_t)'p':
        case (uint8_t)'P':
        {
            Motor_PrintStatus();
            return;
        }

        case (uint8_t)'h':
        case (uint8_t)'H':
        {
            Motor_PrintTestHelp();
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

    if (motionCommand)
    {
        Motor_StartTestTimeout(nowMs);
    }

    Motor_PrintStatus();
}