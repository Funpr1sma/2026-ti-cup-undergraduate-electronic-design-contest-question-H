#include "clock.h"
#include "Key.h"

void SysTick_Handler(void)
{
    tick_ms++;
    Key_Tick();
}
