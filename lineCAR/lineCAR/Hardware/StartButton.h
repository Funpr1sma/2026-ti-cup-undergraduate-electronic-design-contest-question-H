#ifndef HARDWARE_START_BUTTON_H_
#define HARDWARE_START_BUTTON_H_

#include <stdint.h>

/*
 * Three independent active-low buttons. Each physical button is connected
 * between its GPIO and GND; SysConfig enables the internal pull-up:
 *
 *   PB14: requirement 2 - fast lap and stop at A.
 *   PB11: requirements 4/5 - continuous line following; balance target is O.
 *   PB10: requirement 6 - continuous line following; the balance board
 *         captures the current camera ball-center coordinate as its target.
 *
 * A press while the car is active is an emergency stop. PB11/PB10 also start
 * a non-blocking full-duplex readiness handshake with the balance board.
 */
void StartButton_Init(uint32_t nowMs);
void StartButton_Task(uint32_t nowMs);

#endif /* HARDWARE_START_BUTTON_H_ */
