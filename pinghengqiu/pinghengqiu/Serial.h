#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

/* UART0: PB0 TX, PB1 RX, 115200, 8N1. */
void Serial_Init(void);

/* Finite-wait transmit: a serial fault must not block the control loop forever. */
void Serial_SendByte(uint8_t data);
void Serial_SendArray(const uint8_t *data, uint16_t length);
void Serial_SendString(const char *text);
void Serial_Printf(const char *format, ...);

/* Non-blocking access to the UART0 receive ring buffer. */
bool Serial_TryReadByte(uint8_t *data);

uint32_t Serial_GetRxOverflowCount(void);
uint32_t Serial_GetRxIrqCount(void);
uint32_t Serial_GetTxDropCount(void);

#endif /* SERIAL_H_ */
