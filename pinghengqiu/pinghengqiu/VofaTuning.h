#ifndef VOFA_TUNING_H_
#define VOFA_TUNING_H_

#include <stdint.h>

/* UART0/VOFA+ FireWater command and waveform interface. */
void VofaTuning_Init(uint32_t now_ms);
void VofaTuning_Process(uint32_t now_ms);

#endif /* VOFA_TUNING_H_ */
