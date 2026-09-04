#ifndef KEY_H_
#define KEY_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define KEY_COUNT       4U
#define Key_Count       KEY_COUNT

/* All four inputs use internal pull-ups; a button press shorts the pin to GND. */
#define KEY_1           0U  /* PB21: spare */
#define KEY_2           1U  /* PB4: return to configured default level and zero */
#define KEY_3           2U  /* PB5: save current track position as 0 degrees */
#define KEY_4           3U  /* PB8: requirement-3 start / second-press stop */

#define KEY_PRESSED     1U
#define KEY_UNPRESSED   0U

#define KEY_HOLD        0x01U
#define KEY_DOWN        0x02U
#define KEY_DOWM        KEY_DOWN  /* compatibility with the old misspelled macro */
#define KEY_UP          0x04U
#define KEY_SINGLE      0x08U
#define KEY_DOUBLE      0x10U
#define KEY_LONG        0x20U
#define KEY_REPEAT      0x40U

void Key_Init(void);
void Key_Tick(void);                 /* call from the 1 ms SysTick interrupt */
uint8_t Key_GetState(uint8_t index); /* raw active-low input converted to 0/1 */
uint8_t Key_Check(uint8_t index, uint8_t event_flag);
uint8_t Key(uint16_t index);         /* compatibility: returns one single-click event */
uint8_t Key_Sensor(GPIO_Regs *gpio, uint32_t pins);

#endif /* KEY_H_ */
