#ifndef HARDWARE_ENCODER_H_
#define HARDWARE_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * 双路编码器映射：
 *
 * MOTOR1：
 *     A = PA25
 *     B = PA14
 *     GPIO双边沿中断软件正交解码
 *
 * MOTOR2：
 *     A = PA26 / TIMG8_C0
 *     B = PA27 / TIMG8_C1
 *     TIMG8硬件QEI
 */

typedef enum
{
    ENCODER_ID_MOTOR1 = 0,
    ENCODER_ID_MOTOR2 = 1

} EncoderId_t;

typedef struct
{
    /**
     * 当前底层计数源。
     *
     * MOTOR1：软件正交累计计数；
     * MOTOR2：TIMG8当前16位计数值。
     */
    int32_t sourceCount;

    /**
     * 最近一次采样周期内的计数变化量。
     */
    int32_t deltaCount;

    /**
     * 经过方向修正后的软件累计计数。
     */
    int32_t totalCount;

    /**
     * 当前速度，单位Counts Per Second。
     */
    int32_t cps;

    /**
     * 本次测速实际使用的时间间隔。
     */
    uint32_t sampleIntervalMs;

    /**
     * MOTOR1软件解码非法跳变数量。
     *
     * MOTOR2固定为0。
     */
    uint32_t invalidTransitions;

} EncoderData_t;

/**
 * @brief 初始化双路编码器。
 */
void Encoder_Init(void);

/**
 * @brief 清零软件累计计数和测速状态。
 */
void Encoder_ResetAll(void);

/**
 * @brief 使用指定时间间隔立即更新双路编码器。
 */
void Encoder_Update(uint32_t elapsedMs);

/**
 * @brief 编码器20 ms采样任务。
 *
 * @return true：本次调用完成了一次新测速；
 *         false：尚未到测速周期。
 */
bool Encoder_Task(uint32_t nowMs);

/**
 * @brief 获取指定编码器最近一次数据。
 */
EncoderData_t Encoder_GetData(EncoderId_t encoder);

/**
 * @brief 获取MOTOR1当前CPS。
 */
int32_t Encoder_GetMotor1Cps(void);

/**
 * @brief 获取MOTOR2当前CPS。
 */
int32_t Encoder_GetMotor2Cps(void);

/**
 * @brief 获取MOTOR1软件正交累计计数。
 */
int32_t Encoder_GetMotor1SoftwareCount(void);

/**
 * @brief 通过串口输出当前双路编码器数据。
 */
void Encoder_PrintStatus(void);

#endif /* HARDWARE_ENCODER_H_ */