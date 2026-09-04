#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

#include <stdint.h>

/**
 * @brief 扩展板上的两个电机接口。
 *
 * MOTOR_ID_1：
 *     扩展板MOTOR1
 *     PWMB = PA13 / TIMG0_C1
 *     BIN1 = PA16
 *     BIN2 = PB24
 *
 * MOTOR_ID_2：
 *     扩展板MOTOR2
 *     PWMA = PA12 / TIMG0_C0
 *     AIN1 = PB17
 *     AIN2 = PB19
 */
typedef enum
{
    MOTOR_ID_1 = 0,
    MOTOR_ID_2 = 1

} MotorId_t;

/**
 * @brief 电机当前工作模式。
 */
typedef enum
{
    /**
     * IN1=0，IN2=0，PWM=0。
     * H桥输出为高阻状态，电机依靠惯性停止。
     */
    MOTOR_MODE_COAST = 0,

    /**
     * IN1=1，IN2=1，PWM=0。
     * 电机短路制动。
     */
    MOTOR_MODE_BRAKE,

    /**
     * 电机正转。
     */
    MOTOR_MODE_FORWARD,

    /**
     * 电机反转。
     */
    MOTOR_MODE_REVERSE

} MotorMode_t;

/**
 * @brief 电机当前的软件状态。
 */
typedef struct
{
    /**
     * 当前工作模式。
     */
    MotorMode_t mode;

    /**
     * 当前PWM占空比，范围0～100。
     */
    uint8_t dutyPercent;

    /**
     * 当前有符号控制量。
     *
     * 正数：正转；
     * 负数：反转；
     * 0：停止。
     */
    int16_t signedCommand;

} MotorStatus_t;

/**
 * @brief 初始化双电机模块。
 *
 * 函数内部会初始化方向模块和PWM模块，
 * 初始化完成后两个电机均处于制动状态。
 */
void Motor_Init(void);

/**
 * @brief 使用有符号百分比控制电机。
 *
 * @param motor 电机编号。
 * @param speedPercent 控制量，范围-100～+100。
 *
 * speedPercent > 0：正转；
 * speedPercent < 0：反转；
 * speedPercent = 0：短路制动。
 */
void Motor_SetSpeed(
    MotorId_t motor,
    int16_t speedPercent
);

/**
 * @brief 指定电机正转。
 *
 * @param dutyPercent 占空比，范围0～100。
 */
void Motor_Forward(
    MotorId_t motor,
    uint8_t dutyPercent
);

/**
 * @brief 指定电机反转。
 *
 * @param dutyPercent 占空比，范围0～100。
 */
void Motor_Reverse(
    MotorId_t motor,
    uint8_t dutyPercent
);

/**
 * @brief 指定电机短路制动。
 */
void Motor_Brake(MotorId_t motor);

/**
 * @brief 指定电机高阻滑行停止。
 */
void Motor_Coast(MotorId_t motor);

/**
 * @brief 两个电机同时短路制动。
 */
void Motor_BrakeAll(void);

/**
 * @brief 两个电机同时高阻滑行停止。
 */
void Motor_CoastAll(void);

/**
 * @brief 获取指定电机的当前软件状态。
 */
MotorStatus_t Motor_GetStatus(MotorId_t motor);

/**
 * @brief 输出阶段6串口测试命令。
 */
void Motor_PrintTestHelp(void);

/**
 * @brief 阶段6电机测试任务。
 *
 * 应在主循环中持续调用。
 *
 * 运动命令执行1500 ms后会自动制动。
 *
 * @param nowMs 当前系统毫秒数。
 */
void Motor_TestTask(uint32_t nowMs);

#endif /* HARDWARE_MOTOR_H_ */