#ifndef HARDWARE_VOFA_TUNING_H_
#define HARDWARE_VOFA_TUNING_H_

#include <stdint.h>

/*
 * 初始化VOFA+ FireWater在线调参模块。
 *
 * 使用UART0：PA10 TX、PA11 RX、115200 8N1。
 */
void VofaTuning_Init(uint32_t nowMs);

/*
 * 周期任务：
 * 1. 接收并解析以换行结束的ASCII命令；
 * 2. 按设定周期输出FireWater CSV数据帧。
 */
void VofaTuning_Task(uint32_t nowMs);

#endif /* HARDWARE_VOFA_TUNING_H_ */
