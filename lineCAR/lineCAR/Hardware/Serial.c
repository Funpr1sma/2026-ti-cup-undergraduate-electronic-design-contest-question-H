#include "Serial.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 环形缓冲区长度必须是2的整数次幂。
 *
 * 256字节足以容纳多条VOFA+短命令，
 * 同时不会明显增加MSPM0G3507的RAM占用。
 */
#define SERIAL_RX_BUFFER_SIZE        (256U)
#define SERIAL_RX_BUFFER_MASK        \
    (SERIAL_RX_BUFFER_SIZE - 1U)

/*
 * UART is a debugging interface only. Never allow a missing or abnormal
 * serial connection to block the vehicle control loop indefinitely.
 */
#define SERIAL_TX_WAIT_LIMIT_LOOPS   (20000U)
#define SERIAL_RX_MAX_BYTES_PER_IRQ  (32U)

#if ((SERIAL_RX_BUFFER_SIZE & SERIAL_RX_BUFFER_MASK) != 0U)
#error "SERIAL_RX_BUFFER_SIZE must be a power of two"
#endif

static volatile uint8_t g_rxBuffer[
    SERIAL_RX_BUFFER_SIZE
];

/* ISR只写head，主循环只写tail。 */
static volatile uint16_t g_rxHead = 0U;
static volatile uint16_t g_rxTail = 0U;

static volatile uint32_t g_rxOverflowCount = 0U;
static volatile uint32_t g_rxIrqCount = 0U;
static volatile uint32_t g_txDropCount = 0U;


void Serial_Init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;

    g_rxOverflowCount = 0U;
    g_rxIrqCount = 0U;
    g_txDropCount = 0U;

    /*
     * An unplugged USB-to-UART adapter leaves PA11/RX without an external idle
     * high level. Force the UART RX pin to peripheral mode with an internal
     * pull-up so that an open connector cannot generate continuous noise RX
     * interrupts and starve the button/control tasks.
     */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_UART_0_IOMUX_RX,
        GPIO_UART_0_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /* Discard any byte/noise captured before the pull-up became effective. */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST))
    {
        (void)DL_UART_Main_receiveData(UART_0_INST);
    }

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}


void Serial_SendByte(uint8_t data)
{
    uint32_t waitLoops = SERIAL_TX_WAIT_LIMIT_LOOPS;

    /*
     * Use the FIFO-check API with a finite wait instead of the DriverLib
     * infinite-blocking API. Under normal 115200-baud operation this behaves
     * like a regular blocking write. If UART hardware is unavailable or
     * abnormal, the byte is dropped and vehicle control continues.
     */
    while (!DL_UART_Main_transmitDataCheck(UART_0_INST, data))
    {
        if (waitLoops == 0U)
        {
            g_txDropCount++;
            return;
        }
        waitLoops--;
    }
}


void Serial_SendString(const char *text)
{
    if (text == 0)
    {
        return;
    }

    while (*text != '\0')
    {
        Serial_SendByte((uint8_t)(*text));
        text++;
    }
}


bool Serial_TryReadByte(uint8_t *data)
{
    uint16_t tail;

    if (data == 0)
    {
        return false;
    }

    tail = g_rxTail;

    if (tail == g_rxHead)
    {
        return false;
    }

    *data = g_rxBuffer[tail];

    g_rxTail = (uint16_t)(
        (tail + 1U) &
        SERIAL_RX_BUFFER_MASK
    );

    return true;
}


uint32_t Serial_GetRxOverflowCount(void)
{
    return g_rxOverflowCount;
}


uint32_t Serial_GetRxIrqCount(void)
{
    return g_rxIrqCount;
}


uint32_t Serial_GetTxDropCount(void)
{
    return g_txDropCount;
}


void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint32_t drained = 0U;

            g_rxIrqCount++;

            /* Bound one ISR entry even if RX is electrically noisy. */
            while ((!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) &&
                   (drained < SERIAL_RX_MAX_BYTES_PER_IRQ))
            {
                uint8_t receivedData;
                uint16_t head;
                uint16_t nextHead;

                receivedData = DL_UART_Main_receiveData(UART_0_INST);
                drained++;

                head = g_rxHead;

                nextHead = (uint16_t)(
                    (head + 1U) &
                    SERIAL_RX_BUFFER_MASK
                );

                if (nextHead == g_rxTail)
                {
                    /* 缓冲区已满，丢弃最新字节。 */
                    g_rxOverflowCount++;
                }
                else
                {
                    g_rxBuffer[head] = receivedData;
                    g_rxHead = nextHead;
                }
            }

            break;
        }

        default:
            break;
    }
}
