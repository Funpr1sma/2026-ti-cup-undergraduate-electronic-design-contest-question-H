/**
  ******************************************************************************
  * @file    closed_loop.h
  * @brief   单轴步进电机位置闭环控制接口
  ******************************************************************************
  * 使用方法：
  * 1. 电机和编码器初始化完成后调用一次CL_Init()，上电位置会作为软件零点；
  * 2. 需要转动时调用CL_SetTargetAngle()设置相对零点的目标角度；
  * 3. 每隔CL_PERIOD_MS调用一次CL_Process()，函数会自动分段发脉冲并检查反馈；
  * 4. CL_IsReached()用于判断到位，CL_GetFault()用于读取故障；
  * 5. 出现故障时先排查接线和方向，再调用CL_ClearFault()清除故障。
  *
  * 控制过程：计算误差 -> 发一小段STEP -> 等待完成 -> 检查A/B反馈。
  * 误差大时提高频率，接近目标时降低频率；连续进入容差区后判定到位。
  ******************************************************************************
  */
#ifndef CLOSED_LOOP_H
#define CLOSED_LOOP_H

#include <stdint.h>
#include "motor.h"

/* 闭环故障码：无故障、无编码器反馈、方向相反、底层驱动失败。 */
typedef enum {
    CL_FAULT_NONE = 0,
    CL_FAULT_NO_ENCODER,
    CL_FAULT_DIRECTION,
    CL_FAULT_DRIVER
} CL_Fault_t;

/* 提供给串口打印和上位机显示的一次性状态快照。 */
typedef struct {
    int32_t current_count;
    int32_t target_count;
    int32_t error_count;
    float current_angle_deg;
    float target_angle_deg;
    uint8_t active;
    uint8_t reached;
    CL_Fault_t fault;
} CL_Snapshot_t;

/* 初始化闭环状态并将上电位置设为软件零点。 */
void CL_Init(void);
/* 每5ms调用一次；函数非阻塞，电机忙时会等待当前脉冲段结束。 */
void CL_Process(void);
/* 设置相对软件零点的目标角度；有故障时拒绝新目标。 */
MotorStatus_t CL_SetTargetAngle(MotorAxis_t axis, float target_deg);
void CL_SetZero(MotorAxis_t axis);
void CL_SetZeroAll(void);
void CL_Stop(MotorAxis_t axis);
void CL_StopAll(void);
/* 清故障后目标锁定到当前位置，避免电机突然补偿旧目标。 */
void CL_ClearFault(MotorAxis_t axis);
MotorStatus_t CL_TogglePositiveDirLevel(MotorAxis_t axis);
uint8_t CL_IsReached(MotorAxis_t axis);
CL_Fault_t CL_GetFault(MotorAxis_t axis);
float CL_GetCurrentAngle(MotorAxis_t axis);
void CL_GetSnapshot(MotorAxis_t axis, CL_Snapshot_t *snapshot);

#endif
