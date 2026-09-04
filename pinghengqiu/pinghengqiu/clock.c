#include "ti_msp_dl_config.h"
#include "clock.h"
volatile unsigned long tick_ms;
int mspm0_delay_ms(unsigned long num_ms)
{
    unsigned long start_time;
    start_time = tick_ms;
    while ((unsigned long)(tick_ms - start_time) < num_ms) {
    }
    return 0;
}
int mspm0_get_clock_ms(unsigned long *count)
{
    if (count == 0) {
        return 1;
    }
    *count = tick_ms;
    return 0;
}
void SysTick_Init(void)
{
    tick_ms = 0UL;
    /*
     * CPUCLK_FREQ / 1000：
     * 每1ms进入一次SysTick_Handler。
     */
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    NVIC_SetPriority(SysTick_IRQn, 0);
}