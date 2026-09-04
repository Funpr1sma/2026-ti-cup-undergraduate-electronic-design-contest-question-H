/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   MS42CG单轴编码器A/B增量计数、Z圈数和PWM绝对角度接口
  ******************************************************************************
  * A=PA1/TIMG8_CCP0，B=PA0/TIMG8_CCP1，由TIMG8硬件QEI四倍频计数；
  * PWM=PB20/TIMG12_CCP0，由组合捕获同时测量周期和高电平时间。
  * Z=PA25/GPIO上升沿中断，每次经过机械基准点时按QEI方向累计有符号圈数。
  ******************************************************************************
  */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum { ENCODER_AXIS_X = 0 } EncoderAxis_t;

/* 初始化QEI、PWM捕获和软件累计状态。 */
void Encoder_Init(void);
/* 固定周期调用：扩展16位QEI计数器并维护PWM信号超时。 */
void Encoder_Tick(uint32_t elapsed_ms);
/* 读取相对软件零点的多圈有符号计数。 */
int32_t Encoder_GetCount(EncoderAxis_t axis);
float Encoder_GetAngle(EncoderAxis_t axis);
/* Power-up-relative multi-turn position; unaffected by Encoder_SetZero(). */
int32_t Encoder_GetRawCount(EncoderAxis_t axis);
float Encoder_GetRawAngle(EncoderAxis_t axis);
/* 根据两次调用间的计数差计算角速度，单位为度/秒。 */
float Encoder_CalcSpeedDps(EncoderAxis_t axis, uint32_t period_ms);
void Encoder_SetZero(EncoderAxis_t axis);
void Encoder_SetZeroAll(void);
/* 读取物理Z脉冲累计的有符号净圈数：正方向+1，反方向-1。 */
int32_t Encoder_GetZCount(EncoderAxis_t axis);
/* 读取最近一次Z上升沿时的A/B相对计数，用于检查每圈是否接近4000计数。 */
int32_t Encoder_GetCountAtLastZ(EncoderAxis_t axis);
/* 读取0~360度PWM绝对角度；返回0表示未接线、超时或捕获数据无效。 */
uint8_t Encoder_GetPwmAngle(EncoderAxis_t axis, float *angle_deg);

#endif
