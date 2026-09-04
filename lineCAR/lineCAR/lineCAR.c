#include "ti_msp_dl_config.h"

#include "Config/CarConfig.h"
#include "Hardware/CarControl.h"
#include "Hardware/Encoder.h"
#include "Hardware/GraySensor.h"
#include "Hardware/LineFollow.h"
#include "Hardware/Motor.h"
#include "Hardware/OLED.h"
#include "Hardware/Serial.h"
#include "Hardware/SpeedPI.h"
#include "Hardware/StartButton.h"
#include "Hardware/VofaTuning.h"
#include "System/Timer.h"
#include "CarMotionLink.h"
#include "MotionPlanner.h"

#include <stdint.h>
#include <stdbool.h>
// #define TEST_TARGET_CPS       (2000L)
// #define TEST_RUN_TIME_MS      (3000U)
// #define TEST_READY_TIME_MS    (4000U)

// int main(void)
// {
//     uint32_t nowMs;
//     uint32_t readyStartMs;
//     uint32_t runStartMs = 0U;
//     SpeedPIStatus_t speedStatus;

//     SYSCFG_DL_init();
//     Serial_Init();
//     Timer_Init();

//     Motor_Init();
//     Encoder_Init();
//     SpeedPI_Init();

//     SpeedPI_StopAll();

//     Serial_SendString(
//         "\r\n================================\r\n"
//         " Fixed speed distance test\r\n"
//         " Target: 1000 CPS\r\n"
//         " Run time: 1000 ms\r\n"
//         " Start after 3 seconds\r\n"
//         "================================\r\n"
//     );

//     /*
//      * 等待3秒，方便放好小车并标记起点。
//      */
//     readyStartMs = Timer_GetMillis();

//     while ((uint32_t)(Timer_GetMillis() - readyStartMs) <
//            TEST_READY_TIME_MS)
//     {
//         SpeedPI_ControlTask(Timer_GetMillis());
//     }

//     /*
//      * 清零编码器和PI状态，然后开始固定速度运行。
//      */
//     Encoder_ResetAll();
//     SpeedPI_ResetAllIntegrals();

//     /*
//      * 如果起步困难，可以把0U改为25U。
//      */
//     SpeedPI_SetAllStartupMinimumPercent(0U);

//     SpeedPI_SetTargets(
//         TEST_TARGET_CPS,
//         TEST_TARGET_CPS
//     );

//     Serial_SendString("TEST START\r\n");

//     while (1)
//     {
//         nowMs = Timer_GetMillis();

//         /*
//          * 必须持续调用，内部每20 ms进行一次编码器采样
//          * 和两轮速度PI控制。
//          */
//         SpeedPI_ControlTask(nowMs);

//         /*
//          * 从PI第一次真正输出PWM时开始计时，
//          * 避免初始化编码器造成少跑一个控制周期。
//          */
//         if (runStartMs == 0U)
//         {
//             speedStatus = SpeedPI_GetStatus();

//             if ((speedStatus.motor1.outputPercent != 0) ||
//                 (speedStatus.motor2.outputPercent != 0))
//             {
//                 runStartMs = nowMs;
//             }
//         }
//         else if ((uint32_t)(nowMs - runStartMs) >=
//                  TEST_RUN_TIME_MS)
//         {
//             break;
//         }
//     }

//     /*
//      * 1秒到达，主动制动。
//      */
//     SpeedPI_StopAll();

//     Serial_SendString(
//         "TEST STOP\r\n"
//         "Measure the travelled distance now.\r\n"
//     );

//     /*
//      * 永久停车，防止再次启动。
//      */
//     while (1)
//     {
//         __WFI();
//     }
// }


int main(void)
{   
    LineFollowStatus_t lineStatus;
    bool ready;
    bool running;
    bool emergency;
    bool preTilt;
    bool balanceMode;
    uint32_t nowMs;
    uint32_t lastLedMs;
    CarControlStatus_t carStatus;

    SYSCFG_DL_init();
    Serial_Init();
    Timer_Init();

        Serial_SendString(
        "\n==============================\n"
        " LineCAR H2026 Req2 + Req45 + Req6\n"
        " Keys PB14=req2 PB11=req45 PB10=req6 active-low to GND\n"
        " UART0 PA10/PA11 115200 8N1\n"
        "==============================\n"
    );

    Motor_Init();
    Encoder_Init();
    SpeedPI_Init();
    GraySensor_Init();
    LineFollow_Init();

    nowMs = Timer_GetMillis();

    MotionPlanner_Init(nowMs);
    CarMotionLink_Init(nowMs);

    CarControl_Init();
    StartButton_Init(nowMs);
    VofaTuning_Init(nowMs);

#if CAR_ENABLE_OLED != 0U
    OLED_Init(nowMs);
#endif

    lastLedMs = nowMs;

        while (1)
        {
            nowMs = Timer_GetMillis();

            if ((uint32_t)(nowMs - lastLedMs) >= 500U)
            {
                lastLedMs = nowMs;

                DL_GPIO_togglePins(
                    GPIO_TEST_PORT,
                    GPIO_TEST_LED_PIN
                );
            }

            StartButton_Task(nowMs);
            CarControl_Task(nowMs);

            /*
             * Service the safety-critical inter-board link before optional
             * OLED and debug UART work. Heavy VOFA output can then delay only
             * the next main-loop pass, not the current acceleration frame.
             */

            /* 在这里 */
            carStatus = CarControl_GetStatus();
            lineStatus = LineFollow_GetStatus();

            /*
            * ARMING表示：
            *
            * 1. 已经按下启动按钮；
            * 2. 小车正在确认起始标志；
            * 3. 尚未正式开始运动。
            *
            * 此时通知杆球板提前准备。
            */
            ready = (carStatus.state == CAR_STATE_ARMING) ||
                (carStatus.state == CAR_STATE_PRETILT);
            preTilt = (carStatus.state == CAR_STATE_PRETILT);

            running =
                ((carStatus.state == CAR_STATE_RUNNING) ||
                (carStatus.state == CAR_STATE_PASSING_FINISH)) &&
                (
                    CarControl_IsStraightTuning() ||
                    (
                        lineStatus.enabled &&
                        ((lineStatus.leftTargetCps != 0) ||
                        (lineStatus.rightTargetCps != 0))
                    )
                );

            emergency =
            (carStatus.state == CAR_STATE_STOPPING) ||
            (carStatus.state == CAR_STATE_FAULT) ||
            (
                !CarControl_IsStraightTuning() &&
                (carStatus.state == CAR_STATE_RUNNING) &&
                lineStatus.enabled &&
                !lineStatus.lineValid &&
                !lineStatus.replayingLastState
            );
            balanceMode = CarControl_IsBalanceMission();
            CarMotionLink_Task(
                nowMs,
                ready,
                running,
                emergency,
                preTilt,
                balanceMode,
                CarControl_IsRequirement6());
        #if CAR_ENABLE_OLED != 0U
            OLED_Task(nowMs);
        #endif

            /* Debug command parsing/plotting is intentionally last. */
            VofaTuning_Task(nowMs);
        }
}