#ifndef HARDWARE_MOTOR_DIRECTION_H_
#define HARDWARE_MOTOR_DIRECTION_H_

#include <stdint.h>

/**
 * TB6612通道与扩展板接口对应关系：
 *
 * MOTOR_CHANNEL_A：
 *     扩展板MOTOR2
 *     PWMA = PA12
 *     AIN1 = PB17
 *     AIN2 = PB19
 *
 * MOTOR_CHANNEL_B：
 *     扩展板MOTOR1
 *     PWMB = PA13
 *     BIN1 = PA16
 *     BIN2 = PB24
 *
 * 当前阶段不定义左右轮。
 */

/**
 * @brief TB6612方向输入状态。
 */
typedef enum
{
    /**
     * IN1=0，IN2=0。
     *
     * TB6612通常处于停止/滑行状态。
     */
    MOTOR_DIRECTION_COAST = 0,

    /**
     * IN1=1，IN2=0。
     *
     * 暂时称为方向1，阶段6再判断是否为小车前进。
     */
    MOTOR_DIRECTION_INPUT1_HIGH,

    /**
     * IN1=0，IN2=1。
     *
     * 暂时称为方向2。
     */
    MOTOR_DIRECTION_INPUT2_HIGH,

    /**
     * IN1=1，IN2=1。
     *
     * TB6612通常为短路制动。
     * 阶段4测试命令不会使用该状态。
     */
    MOTOR_DIRECTION_BRAKE

} MotorDirectionState_t;

/**
 * @brief 初始化电机方向模块。
 *
 * 初始化后：
 * AIN1、AIN2、BIN1、BIN2全部为低电平。
 */
void MotorDirection_Init(void);

/**
 * @brief 设置通道A，也就是扩展板MOTOR2的方向输入。
 */
void MotorDirection_SetChannelA(MotorDirectionState_t state);

/**
 * @brief 设置通道B，也就是扩展板MOTOR1的方向输入。
 */
void MotorDirection_SetChannelB(MotorDirectionState_t state);

/**
 * @brief 将两个通道的方向输入全部拉低。
 */
void MotorDirection_StopAll(void);

/**
 * @brief 获取通道A的软件记录状态。
 */
MotorDirectionState_t MotorDirection_GetChannelA(void);

/**
 * @brief 获取通道B的软件记录状态。
 */
MotorDirectionState_t MotorDirection_GetChannelB(void);

/**
 * @brief 输出阶段4串口测试命令帮助。
 */
void MotorDirection_PrintTestHelp(void);

/**
 * @brief 阶段4串口测试任务。
 *
 * 在主循环中持续调用。函数会读取一个串口字符，
 * 并根据命令切换四个方向GPIO。
 */
void MotorDirection_TestTask(void);

#endif /* HARDWARE_MOTOR_DIRECTION_H_ */