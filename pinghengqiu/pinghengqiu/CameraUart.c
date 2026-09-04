#include "CameraUart.h"

#include <stddef.h>
#include <string.h>

#include "ball_link.h"
#include "ti_msp_dl_config.h"

#define CAMERA_UART_MAX_DRAIN_PER_CALL 128U
#define CAMERA_UART_TX_WAIT_LIMIT      100000U

static volatile CameraUartDiagnostics_t s_camera_uart;
static uint8_t s_pong_match_index;

static void CameraUart_RecordRecent(uint8_t byte)
{
    uint8_t count = s_camera_uart.recent_count;

    if (count < CAMERA_UART_RECENT_BYTE_COUNT) {
        s_camera_uart.recent[count] = byte;
        s_camera_uart.recent_count = (uint8_t)(count + 1U);
    } else {
        uint32_t index;
        for (index = 1U; index < CAMERA_UART_RECENT_BYTE_COUNT; index++) {
            s_camera_uart.recent[index - 1U] = s_camera_uart.recent[index];
        }
        s_camera_uart.recent[CAMERA_UART_RECENT_BYTE_COUNT - 1U] = byte;
    }
}

static void CameraUart_CheckPong(uint8_t byte)
{
    static const uint8_t pong[] = {'$', 'P', 'O', 'N', 'G', '*'};

    if (byte == pong[s_pong_match_index]) {
        s_pong_match_index++;
        if (s_pong_match_index >= sizeof(pong)) {
            s_camera_uart.pong_count++;
            s_pong_match_index = 0U;
        }
    } else {
        s_pong_match_index = (byte == pong[0]) ? 1U : 0U;
    }
}

static void CameraUart_ProcessByte(uint8_t byte)
{
    s_camera_uart.rx_byte_count++;
    s_camera_uart.last_byte = byte;
    CameraUart_RecordRecent(byte);
    CameraUart_CheckPong(byte);
    (void)BallLink_RxByte(byte);
}

static uint32_t CameraUart_DrainRx(uint32_t limit)
{
    uint32_t drained = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST) && drained < limit) {
        CameraUart_ProcessByte(DL_UART_Main_receiveData(UART_2_INST));
        drained++;
    }

    if (drained > s_camera_uart.max_drain_bytes) {
        s_camera_uart.max_drain_bytes = drained;
    }

    return drained;
}

void CameraUart_ResetDiagnostics(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset((void *)&s_camera_uart, 0, sizeof(s_camera_uart));
    s_pong_match_index = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

void CameraUart_Init(void)
{
    CameraUart_ResetDiagnostics();

    /* Discard any stale byte left by a previous debug session. */
    (void)CameraUart_DrainRx(CAMERA_UART_MAX_DRAIN_PER_CALL);

    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void CameraUart_Poll(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t drained;

    /* Prevent the UART2 ISR from draining the same FIFO concurrently. */
    __disable_irq();
    drained = CameraUart_DrainRx(CAMERA_UART_MAX_DRAIN_PER_CALL);
    if (drained > 0U) {
        s_camera_uart.poll_drain_count++;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t CameraUart_SendString(const char *text)
{
    if (text == 0) {
        return 0U;
    }

    while (*text != '\0') {
        uint32_t wait_count = 0U;
        uint8_t byte = (uint8_t)*text;

        while (!DL_UART_Main_transmitDataCheck(UART_2_INST, byte)) {
            wait_count++;
            if (wait_count >= CAMERA_UART_TX_WAIT_LIMIT) {
                s_camera_uart.tx_drop_count++;
                return 0U;
            }
        }

        s_camera_uart.tx_byte_count++;
        text++;
    }

    return 1U;
}

void CameraUart_GetDiagnostics(CameraUartDiagnostics_t *diagnostics)
{
    uint32_t primask;

    if (diagnostics == 0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = s_camera_uart;
    if (primask == 0U) {
        __enable_irq();
    }
}

void UART_2_INST_IRQHandler(void)
{
    s_camera_uart.irq_count++;

    /*
     * Drain the FIFO regardless of the exact UART interrupt index. This also
     * handles RX timeout/error combinations that previously fell through the
     * switch statement without consuming received bytes.
     */
    (void)DL_UART_Main_getPendingInterrupt(UART_2_INST);
    (void)CameraUart_DrainRx(CAMERA_UART_MAX_DRAIN_PER_CALL);
}
