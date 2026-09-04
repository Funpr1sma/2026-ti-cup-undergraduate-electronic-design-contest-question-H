#include "ti_msp_dl_config.h"

#include <stdint.h>

#include "CameraUart.h"
#include "CarMotionRx.h"
#include "Key.h"
#include "Serial.h"
#include "VofaTuning.h"
#include "ball_balance_control.h"
#include "ball_link.h"
#include "clock.h"
#include "closed_loop.h"
#include "encoder.h"
#include "imu_feedforward.h"
#include "motor.h"

/* The 5 ms timer schedules the slower 20 ms ball-position controller. */
static volatile uint8_t s_balance_process_due;

static uint8_t TakeBalanceProcessRequest(void)
{
    uint32_t primask;
    uint8_t due = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_balance_process_due > 0U) {
        s_balance_process_due--;
        due = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return due;
}

int main(void)
{
    uint32_t last_led_ms;
    uint8_t imu_online;

    SYSCFG_DL_init();
    Key_Init();
    SysTick_Init();
    Serial_Init();

    /*
     * Emit a minimal message before motor, encoder and MPU6050 initialization.
     * If this line is visible, UART0 pin mux, baud rate and TX wiring are good.
     */
    mspm0_delay_ms(50U);
    Serial_SendString("\r\nBOOT UART0 PB0(TX) PB1(RX) 115200 8N1\r\n");

    Motor_Init();
    Encoder_Init();
    CL_Init();
    BallBalance_Init();
    BallLink_Init();
    CameraUart_Init();
    CarMotionRx_Init();

    /*
     * MPU6050 remains independent of the ball controller for now.
     * Its acceleration is acquired and can be plotted in VOFA+, but it is not
     * injected into the balance output until the mechanical direction and gain
     * have been verified on the real vehicle.
     */
    imu_online = ImuFeedforward_Init();

    s_balance_process_due = 0U;

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    VofaTuning_Init((uint32_t)tick_ms);
    Serial_Printf("MPU6050 init=%s; keep the board still and level for calibration\r\n",
                  (imu_online != 0U) ? "OK" : "FAULT");

    last_led_ms = (uint32_t)tick_ms;

    while (1) {
        uint32_t now_ms = (uint32_t)tick_ms;

        /* UART polling fallbacks keep both links alive if an IRQ is missed. */
        CameraUart_Poll();
        CarMotionRx_Poll();
        CarMotionRx_Process(now_ms);

        /*
         * External feed-forward is updated continuously. PB11/PB10 on the car
         * auto-start the matching center/captured-target controller before the
         * car accepts the returned READY acknowledgement.
         */
        BallBalance_UpdateExternalFeedforward(
            CarMotionRx_GetFeedforwardAngleDeg(),
            CarMotionRx_GetAccelerationMps2(),
            CarMotionRx_IsMotionActive(now_ms));

        /* I2C can wait for a few milliseconds, so it intentionally stays in main. */
        ImuFeedforward_Process();

        while (TakeBalanceProcessRequest() != 0U) {
            BallBalance_Process20ms();
        }

        VofaTuning_Process(now_ms);
        CarMotionRx_TxTask(now_ms);

        if ((uint32_t)(now_ms - last_led_ms) >= 500U) {
            last_led_ms = now_ms;
            DL_GPIO_togglePins(LEDs_PORT, LEDs_LED1_PIN);
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    static uint8_t balance_divider;

    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            Encoder_Tick(5U);
            CL_Process();

            balance_divider++;
            if (balance_divider >= 4U) {
                balance_divider = 0U;
                if (s_balance_process_due < 2U) {
                    s_balance_process_due++;
                }
            }
            break;

        default:
            break;
    }
}
