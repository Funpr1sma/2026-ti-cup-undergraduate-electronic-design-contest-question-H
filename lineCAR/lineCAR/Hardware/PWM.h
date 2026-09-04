#ifndef HARDWARE_PWM_H_
#define HARDWARE_PWM_H_

#include <stdint.h>

/**
 * TB6612 PWM通道定义：
 *
 * 通道A：
 *     扩展板MOTOR2
 *     PWMA = PA12
 *     TIMG0_C0
 *
 * 通道B：
 *     扩展板MOTOR1
 *     PWMB = PA13
 *     TIMG0_C1
 */

/**
 * @brief PWM输出通道。
 */
typedef enum
{
    PWM_CHANNEL_A = 0,
    PWM_CHANNEL_B = 1

} PWMChannel_t;

/**
 * @brief 初始化并启动TIMG0双路PWM。
 *
 * 初始化后两路占空比均为0%。
 */
void PWM_Init(void);

/**
 * @brief 设置指定通道的PWM占空比。
 *
 * @param channel PWM_CHANNEL_A或PWM_CHANNEL_B。
 * @param dutyPercent 占空比，范围0～100。
 */
void PWM_SetDuty(
    PWMChannel_t channel,
    uint8_t dutyPercent
);

/**
 * @brief 同时设置两个PWM通道的占空比。
 */
void PWM_SetBothDuty(
    uint8_t channelADuty,
    uint8_t channelBDuty
);

/**
 * @brief 将两路占空比都设置为0%。
 */
void PWM_StopAll(void);

/**
 * @brief 获取通道A当前的软件占空比。
 */
uint8_t PWM_GetChannelADuty(void);

/**
 * @brief 获取通道B当前的软件占空比。
 */
uint8_t PWM_GetChannelBDuty(void);

/**
 * @brief 输出PWM测试命令帮助。
 */
void PWM_PrintTestHelp(void);

/**
 * @brief 阶段5串口测试任务。
 *
 * 应在主循环中持续调用。
 */
void PWM_TestTask(void);

#endif /* HARDWARE_PWM_H_ */