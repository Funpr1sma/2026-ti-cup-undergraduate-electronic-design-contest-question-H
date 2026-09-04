#include "DebugPrint.h"

#include "Serial.h"

void DebugPrint_U32(uint32_t value)
{
    char buffer[10];
    uint32_t index = 0U;

    if (value == 0U)
    {
        Serial_SendByte((uint8_t)'0');
        return;
    }

    while ((value > 0U) && (index < sizeof(buffer)))
    {
        buffer[index++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (index > 0U)
    {
        Serial_SendByte((uint8_t)buffer[--index]);
    }
}

void DebugPrint_I32(int32_t value, bool showPositiveSign)
{
    uint32_t magnitude;

    if (value < 0)
    {
        Serial_SendByte((uint8_t)'-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        if (showPositiveSign && (value > 0))
        {
            Serial_SendByte((uint8_t)'+');
        }
        magnitude = (uint32_t)value;
    }

    DebugPrint_U32(magnitude);
}

void DebugPrint_Bool(bool value)
{
    Serial_SendByte((uint8_t)(value ? '1' : '0'));
}

void DebugPrint_Mask8(uint8_t mask)
{
    uint32_t index;

    for (index = 0U; index < 8U; index++)
    {
        Serial_SendByte((uint8_t)(((mask & (uint8_t)(1U << index)) != 0U) ? '1' : '0'));
    }
}

void DebugPrint_NamedI32(const char *name, int32_t value)
{
    Serial_SendString(name);
    Serial_SendByte((uint8_t)'=');
    DebugPrint_I32(value, false);
}
