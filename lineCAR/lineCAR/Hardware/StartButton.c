#include "StartButton.h"

#include "CarControl.h"
#include "Config/CarConfig.h"
#include "Serial.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BUTTON_REQUIREMENT_2 = 0,
    BUTTON_REQUIREMENT_45,
    BUTTON_REQUIREMENT_6,
    BUTTON_COUNT
} StartButtonId_t;

typedef struct
{
    bool rawPressed;
    bool stablePressed;
    uint32_t rawChangeMs;
} StartButtonState_t;

static StartButtonState_t g_buttons[BUTTON_COUNT];
static uint32_t g_lastTaskMs;

static bool IsPressed(StartButtonId_t button)
{
    switch (button)
    {
        case BUTTON_REQUIREMENT_2:
            return DL_GPIO_readPins(
                START_BUTTON_PORT,
                START_BUTTON_REQUIREMENT2_PIN) == 0U;

        case BUTTON_REQUIREMENT_45:
            return DL_GPIO_readPins(
                START_BUTTON_PORT,
                START_BUTTON_REQUIREMENT45_PIN) == 0U;

        case BUTTON_REQUIREMENT_6:
            return DL_GPIO_readPins(
                START_BUTTON_PORT,
                START_BUTTON_REQUIREMENT6_PIN) == 0U;

        case BUTTON_COUNT:
        default:
            return false;
    }
}

static bool TaskDue(uint32_t nowMs)
{
    if ((uint32_t)(nowMs - g_lastTaskMs) < START_BUTTON_TASK_PERIOD_MS)
    {
        return false;
    }

    g_lastTaskMs += START_BUTTON_TASK_PERIOD_MS;
    if ((uint32_t)(nowMs - g_lastTaskMs) >= START_BUTTON_TASK_PERIOD_MS)
    {
        g_lastTaskMs = nowMs;
    }
    return true;
}

static CarMissionMode_t ButtonMission(StartButtonId_t button)
{
    switch (button)
    {
        case BUTTON_REQUIREMENT_2:
            return CAR_MISSION_STOP_AT_A;

        case BUTTON_REQUIREMENT_45:
            return CAR_MISSION_BALANCE_CENTER;

        case BUTTON_REQUIREMENT_6:
            return CAR_MISSION_BALANCE_CAPTURED_TARGET;

        case BUTTON_COUNT:
        default:
            return CAR_MISSION_STOP_AT_A;
    }
}

static const char *ButtonName(StartButtonId_t button)
{
    switch (button)
    {
        case BUTTON_REQUIREMENT_2: return "REQ2_PB14";
        case BUTTON_REQUIREMENT_45: return "REQ45_PB11";
        case BUTTON_REQUIREMENT_6: return "REQ6_PB10";
        case BUTTON_COUNT:
        default: return "UNKNOWN";
    }
}

static bool CarIsActive(CarState_t state)
{
    return (state == CAR_STATE_ARMING) ||
        (state == CAR_STATE_PRETILT) ||
        (state == CAR_STATE_RUNNING) ||
        (state == CAR_STATE_PASSING_FINISH) ||
        (state == CAR_STATE_STOPPING);
}

static void HandleButtonPress(StartButtonId_t button, uint32_t nowMs)
{
    CarControlStatus_t status = CarControl_GetStatus();
    CarMissionMode_t mission = ButtonMission(button);

    Serial_SendString("BUTTON:");
    Serial_SendString(ButtonName(button));
    Serial_SendString(" pressed\r\n");

    if (CarIsActive(status.state))
    {
        (void)CarControl_Stop(nowMs);
        Serial_SendString("BUTTON:emergency_stop\r\n");
        return;
    }

    if ((status.state == CAR_STATE_FINISHED) ||
        (status.state == CAR_STATE_FAULT))
    {
        if (!CarControl_Reset(nowMs))
        {
            Serial_SendString("BUTTON:reset_rejected\r\n");
            return;
        }
    }

    if (CarControl_StartMode(nowMs, mission))
    {
        Serial_SendString("BUTTON:start mission=");
        if (mission == CAR_MISSION_STOP_AT_A)
        {
            Serial_SendString("2\r\n");
        }
        else if (mission == CAR_MISSION_BALANCE_CENTER)
        {
            Serial_SendString("45_CENTER_CONTINUOUS\r\n");
        }
        else
        {
            Serial_SendString("6_CAPTURE_CURRENT_TARGET\r\n");
        }
    }
    else
    {
        Serial_SendString("BUTTON:start_rejected\r\n");
    }
}

static bool UpdateOneButton(StartButtonId_t button, uint32_t nowMs)
{
    StartButtonState_t *state = &g_buttons[(uint32_t)button];
    bool pressed = IsPressed(button);

    if (pressed != state->rawPressed)
    {
        state->rawPressed = pressed;
        state->rawChangeMs = nowMs;
    }

    if ((pressed != state->stablePressed) &&
        ((uint32_t)(nowMs - state->rawChangeMs) >= START_BUTTON_DEBOUNCE_MS))
    {
        state->stablePressed = pressed;

        /* Trigger once on the debounced press edge. */
        if (pressed)
        {
            return true;
        }
    }

    return false;
}

void StartButton_Init(uint32_t nowMs)
{
    StartButtonId_t button;

    for (button = BUTTON_REQUIREMENT_2;
         button < BUTTON_COUNT;
         button = (StartButtonId_t)((uint32_t)button + 1U))
    {
        bool pressed = IsPressed(button);
        g_buttons[(uint32_t)button].rawPressed = pressed;
        g_buttons[(uint32_t)button].stablePressed = pressed;
        g_buttons[(uint32_t)button].rawChangeMs = nowMs;
    }

    g_lastTaskMs = nowMs;

    Serial_SendString(
        "BUTTON_MAP:PB14=req2,PB11=req45_center,PB10=req6_capture_current,active_low_to_GND\r\n");
}

void StartButton_Task(uint32_t nowMs)
{
    StartButtonId_t button;
    uint8_t pressMask = 0U;
    uint8_t pressCount = 0U;

    if (!TaskDue(nowMs))
    {
        return;
    }

    for (button = BUTTON_REQUIREMENT_2;
         button < BUTTON_COUNT;
         button = (StartButtonId_t)((uint32_t)button + 1U))
    {
        if (UpdateOneButton(button, nowMs))
        {
            pressMask |= (uint8_t)(1U << (uint32_t)button);
            pressCount++;
        }
    }

    if (pressCount == 0U)
    {
        return;
    }

    /* Avoid starting one mission and immediately stopping it if two buttons
     * are pressed at the same time. */
    if (pressCount > 1U)
    {
        Serial_SendString("BUTTON:error_multiple_buttons\r\n");
        return;
    }

    for (button = BUTTON_REQUIREMENT_2;
         button < BUTTON_COUNT;
         button = (StartButtonId_t)((uint32_t)button + 1U))
    {
        if ((pressMask & (uint8_t)(1U << (uint32_t)button)) != 0U)
        {
            HandleButtonPress(button, nowMs);
            return;
        }
    }
}
