#ifndef CAMERA_UART_H_
#define CAMERA_UART_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_UART_RECENT_BYTE_COUNT 8U

typedef struct
{
    uint32_t irq_count;
    uint32_t poll_drain_count;
    uint32_t rx_byte_count;
    uint32_t tx_byte_count;
    uint32_t tx_drop_count;
    uint32_t pong_count;
    uint32_t max_drain_bytes;
    uint8_t last_byte;
    uint8_t recent_count;
    uint8_t recent[CAMERA_UART_RECENT_BYTE_COUNT];
} CameraUartDiagnostics_t;

/* UART2: PB15 TX, PB16 RX, 115200 8N1. */
void CameraUart_Init(void);

/*
 * Polling fallback. Call frequently from main. This makes camera reception
 * continue even if the UART2 RX interrupt is not generated as expected.
 */
void CameraUart_Poll(void);

uint8_t CameraUart_SendString(const char *text);
void CameraUart_GetDiagnostics(CameraUartDiagnostics_t *diagnostics);
void CameraUart_ResetDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_UART_H_ */
