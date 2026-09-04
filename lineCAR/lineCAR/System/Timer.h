#ifndef SYSTEM_TIMER_H_
#define SYSTEM_TIMER_H_

#include <stdint.h>

/**
 * @brief 初始化并启动TIMG7 1 ms系统定时器。
 *
 * 必须先调用SYSCFG_DL_init()。
 */
void Timer_Init(void);

/** @brief 获取系统启动后的毫秒计数。 */
uint32_t Timer_GetMillis(void);

/** @brief 获取定时器中断入口次数，用于诊断。 */
uint32_t Timer_GetIrqCount(void);

#endif /* SYSTEM_TIMER_H_ */
