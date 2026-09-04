#include "Serial.h"
#include "ti_msp_dl_config.h"

#include <stdarg.h>
#include <stdio.h>

#define SERIAL_RX_BUFFER_SIZE        256U
#define SERIAL_RX_BUFFER_MASK        (SERIAL_RX_BUFFER_SIZE - 1U)
#define SERIAL_TX_WAIT_LIMIT_LOOPS   20000U
#define SERIAL_RX_MAX_BYTES_PER_IRQ  32U
#define SERIAL_PRINTF_BUFFER_SIZE    256U

#if ((SERIAL_RX_BUFFER_SIZE & SERIAL_RX_BUFFER_MASK) != 0U)
#error "SERIAL_RX_BUFFER_SIZE must be a power of two"
#endif

static volatile uint8_t s_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overflow_count;
static volatile uint32_t s_rx_irq_count;
static volatile uint32_t s_tx_drop_count;

void Serial_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow_count = 0U;
    s_rx_irq_count = 0U;
    s_tx_drop_count = 0U;

    /*
     * UART0 and PB0/TX are configured by SYSCFG_DL_init(). Re-apply the
     * PB1/RX input features here so an unplugged adapter stays at idle-high.
     */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_UART_0_IOMUX_RX,
        GPIO_UART_0_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        (void)DL_UART_Main_receiveData(UART_0_INST);
    }

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void Serial_SendByte(uint8_t data)
{
    uint32_t wait_loops = SERIAL_TX_WAIT_LIMIT_LOOPS;

    while (!DL_UART_Main_transmitDataCheck(UART_0_INST, data)) {
        if (wait_loops == 0U) {
            s_tx_drop_count++;
            return;
        }
        wait_loops--;
    }
}

void Serial_SendArray(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0U; index < length; index++) {
        Serial_SendByte(data[index]);
    }
}

void Serial_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        Serial_SendByte((uint8_t)*text);
        text++;
    }
}

void Serial_Printf(const char *format, ...)
{
    char buffer[SERIAL_PRINTF_BUFFER_SIZE];
    va_list args;
    int length;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0) {
        return;
    }

    buffer[sizeof(buffer) - 1U] = '\0';
    Serial_SendString(buffer);
}

bool Serial_TryReadByte(uint8_t *data)
{
    uint16_t tail;

    if (data == NULL) {
        return false;
    }

    tail = s_rx_tail;
    if (tail == s_rx_head) {
        return false;
    }

    *data = s_rx_buffer[tail];
    s_rx_tail = (uint16_t)((tail + 1U) & SERIAL_RX_BUFFER_MASK);
    return true;
}

uint32_t Serial_GetRxOverflowCount(void)
{
    return s_rx_overflow_count;
}

uint32_t Serial_GetRxIrqCount(void)
{
    return s_rx_irq_count;
}

uint32_t Serial_GetTxDropCount(void)
{
    return s_tx_drop_count;
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            uint32_t drained = 0U;
            s_rx_irq_count++;

            while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST) &&
                   drained < SERIAL_RX_MAX_BYTES_PER_IRQ) {
                uint8_t received_data;
                uint16_t head;
                uint16_t next_head;

                received_data = DL_UART_Main_receiveData(UART_0_INST);
                drained++;
                head = s_rx_head;
                next_head = (uint16_t)((head + 1U) & SERIAL_RX_BUFFER_MASK);

                if (next_head == s_rx_tail) {
                    s_rx_overflow_count++;
                } else {
                    s_rx_buffer[head] = received_data;
                    s_rx_head = next_head;
                }
            }
            break;
        }

        default:
            break;
    }
}
