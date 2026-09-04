#include "Timer.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_msTick = 0U;
static volatile uint32_t g_irqCount = 0U;

void Timer_Init(void)
{
    g_msTick = 0U;
    g_irqCount = 0U;

    /* 先停止计数器，避免初始化期间产生事件。 */
    DL_TimerG_stopCounter(TIMER_1MS_INST);

    /* 关闭并清理CPU侧中断状态。 */
    NVIC_DisableIRQ(TIMER_1MS_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_1MS_INST_INT_IRQN);

    /* 清除外设侧旧的零事件，再确保零事件中断已开启。 */
    DL_TimerG_disableInterrupt(
        TIMER_1MS_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT
    );

    DL_TimerG_clearInterruptStatus(
        TIMER_1MS_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT
    );

    DL_TimerG_enableInterrupt(
        TIMER_1MS_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT
    );

    NVIC_ClearPendingIRQ(TIMER_1MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_1MS_INST_INT_IRQN);

    /* 显式打开全局中断，避免调试器Restart后PRIMASK保持屏蔽。 */
    __enable_irq();

    DL_TimerG_startCounter(TIMER_1MS_INST);
}

uint32_t Timer_GetMillis(void)
{
    return g_msTick;
}

uint32_t Timer_GetIrqCount(void)
{
    return g_irqCount;
}

void TIMER_1MS_INST_IRQHandler(void)
{
    g_irqCount++;

    if (DL_TimerG_getPendingInterrupt(TIMER_1MS_INST) ==
        DL_TIMER_IIDX_ZERO)
    {
        g_msTick++;
    }

    DL_TimerG_clearInterruptStatus(
        TIMER_1MS_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT
    );
}
