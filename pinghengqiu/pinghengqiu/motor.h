/**
  ******************************************************************************
  * @file    motor.h
  * @brief   D36A单轴STEP/DIR/EN脉冲驱动接口
  ******************************************************************************
  * TIMA1_CCP1从PA24输出50%占空比STEP；PA13控制DIR；PA12控制EN。
  * 定步运行由定时器周期中断精确统计脉冲数，连续运行则关闭计步中断。
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* STEP定时器工作在1MHz计数时钟，频率通过自动重装值换算。 */
#define MOTOR_STEP_TIMER         PWM_0_INST
#define MOTOR_STEP_CC_INDEX      DL_TIMER_CC_1_INDEX
/* STEP定时器工作在1MHz计数时钟，频率通过自动重装值换算。 */
#define MOTOR_STEP_TIMER_CLK_HZ  1000000U
/* D36A细分必须与拨码一致：16细分时一圈需要200*16=3200个STEP脉冲。 */
#define D36A_MICROSTEP           8U
#define MOTOR_STEPS_PER_REV      (200U * D36A_MICROSTEP)
#define MOTOR_MIN_FREQ_HZ        100U
#define MOTOR_MAX_FREQ_HZ        10000U

/* 实际接线：STEP=PA24，DIR=PA13，EN=PA12；当前工程使用高电平使能。 */
#define MOTOR_DIR_PORT           GPIOA
#define MOTOR_DIR_PIN            DL_GPIO_PIN_13
#define MOTOR_EN_PORT            GPIOA
#define MOTOR_EN_PIN             DL_GPIO_PIN_12

/* 轴编号定义；本例程仅使用轴1。 */
typedef enum { MOTOR_AXIS_X = 0 } MotorAxis_t;
typedef enum { MOTOR_OK = 0, MOTOR_ERROR, MOTOR_BUSY } MotorStatus_t;

/* 初始化电机GPIO、停止STEP输出并使能TIMA1中断。 */
void Motor_Init(void);
MotorStatus_t Motor_SetDirection(MotorAxis_t axis, uint8_t high_level);
uint8_t Motor_GetDirection(MotorAxis_t axis);
/* 定步运行：按指定频率输出steps个脉冲；运行中再次启动会返回MOTOR_BUSY。 */
MotorStatus_t Motor_Start(MotorAxis_t axis, uint32_t steps,
                          uint32_t frequency_hz);
/* 连续运行：用于实验2，不统计剩余脉冲，直到调用停止函数。 */
void Motor_StartContinuous(uint32_t frequency_hz, uint8_t high_level);
void Motor_Stop(MotorAxis_t axis);
void Motor_StopAll(void);
uint8_t Motor_IsBusy(MotorAxis_t axis);
uint32_t Motor_GetRemainingSteps(MotorAxis_t axis);
MotorStatus_t Motor_RunContinuous(
    MotorAxis_t axis,
    uint32_t frequency_hz,
    uint8_t high_level);
#endif
