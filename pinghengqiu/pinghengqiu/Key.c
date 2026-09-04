#include "Key.h"

#define KEY_DEBOUNCE_MS       20U
#define KEY_LONG_PRESS_MS     1000U
#define KEY_DOUBLE_CLICK_MS   200U
#define KEY_REPEAT_MS         100U

typedef struct {
    uint8_t stable_state;
    uint8_t candidate_state;
    uint8_t debounce_count;
    uint8_t long_reported;
    uint8_t click_pending;
    uint16_t hold_ms;
    uint16_t click_window_ms;
    uint16_t repeat_ms;
    volatile uint8_t events;
} KeyContext_t;

static KeyContext_t s_keys[KEY_COUNT];

static uint8_t Key_ReadHardware(uint8_t index)
{
    uint32_t pin;

    switch (index) {
        case KEY_1: pin = Keys_Key_0_PIN; break;
        case KEY_2: pin = Keys_Key_1_PIN; break;
        case KEY_3: pin = Keys_Key_2_PIN; break;
        case KEY_4: pin = Keys_Key_3_PIN; break;
        default: return KEY_UNPRESSED;
    }

    return (DL_GPIO_readPins(Keys_PORT, pin) == 0U) ?
        KEY_PRESSED : KEY_UNPRESSED;
}

void Key_Init(void)
{
    uint8_t index;

    for (index = 0U; index < KEY_COUNT; index++) {
        uint8_t state = Key_ReadHardware(index);
        s_keys[index].stable_state = state;
        s_keys[index].candidate_state = state;
        s_keys[index].debounce_count = 0U;
        s_keys[index].long_reported = 0U;
        s_keys[index].click_pending = 0U;
        s_keys[index].hold_ms = 0U;
        s_keys[index].click_window_ms = 0U;
        s_keys[index].repeat_ms = 0U;
        s_keys[index].events = (state == KEY_PRESSED) ? KEY_HOLD : 0U;
    }
}

uint8_t Key_GetState(uint8_t index)
{
    return Key_ReadHardware(index);
}

void Key_Tick(void)
{
    uint8_t index;

    for (index = 0U; index < KEY_COUNT; index++) {
        KeyContext_t *key = &s_keys[index];
        uint8_t raw = Key_ReadHardware(index);

        if (raw == key->candidate_state) {
            if (key->debounce_count < KEY_DEBOUNCE_MS) {
                key->debounce_count++;
            }
        } else {
            key->candidate_state = raw;
            key->debounce_count = 1U;
        }

        if (key->click_pending != 0U && key->click_window_ms > 0U) {
            key->click_window_ms--;
            if (key->click_window_ms == 0U) {
                key->events |= KEY_SINGLE;
                key->click_pending = 0U;
            }
        }

        if (key->debounce_count >= KEY_DEBOUNCE_MS &&
            key->stable_state != key->candidate_state) {
            key->stable_state = key->candidate_state;

            if (key->stable_state == KEY_PRESSED) {
                key->events |= (KEY_HOLD | KEY_DOWN);
                key->hold_ms = 0U;
                key->repeat_ms = 0U;
                key->long_reported = 0U;

                if (key->click_pending != 0U) {
                    key->events |= KEY_DOUBLE;
                    key->click_pending = 0U;
                    key->click_window_ms = 0U;
                }
            } else {
                key->events &= (uint8_t)~KEY_HOLD;
                key->events |= KEY_UP;

                if (key->long_reported == 0U) {
                    key->click_pending = 1U;
                    key->click_window_ms = KEY_DOUBLE_CLICK_MS;
                }
            }
        }

        if (key->stable_state == KEY_PRESSED) {
            if (key->hold_ms < 0xFFFFU) {
                key->hold_ms++;
            }

            if (key->long_reported == 0U &&
                key->hold_ms >= KEY_LONG_PRESS_MS) {
                key->events |= KEY_LONG;
                key->long_reported = 1U;
                key->repeat_ms = 0U;
            } else if (key->long_reported != 0U) {
                key->repeat_ms++;
                if (key->repeat_ms >= KEY_REPEAT_MS) {
                    key->events |= KEY_REPEAT;
                    key->repeat_ms = 0U;
                }
            }
        }
    }
}

uint8_t Key_Check(uint8_t index, uint8_t event_flag)
{
    uint32_t primask;
    uint8_t found;

    if (index >= KEY_COUNT || event_flag == 0U) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    found = (s_keys[index].events & event_flag) != 0U;
    if (event_flag != KEY_HOLD) {
        s_keys[index].events &= (uint8_t)~event_flag;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return found;
}

uint8_t Key(uint16_t index)
{
    if (index >= KEY_COUNT) {
        return 0U;
    }
    return Key_Check((uint8_t)index, KEY_SINGLE);
}

uint8_t Key_Sensor(GPIO_Regs *gpio, uint32_t pins)
{
    return (uint8_t)DL_GPIO_readPins(gpio, pins);
}
