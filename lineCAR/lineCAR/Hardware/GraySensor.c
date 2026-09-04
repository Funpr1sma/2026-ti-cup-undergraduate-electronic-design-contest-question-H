#include "GraySensor.h"

#include "Serial.h"
#include "ti_msp_dl_config.h"

/*
 * 灰度传感器串口报告周期。
 */
#define GRAY_SENSOR_REPORT_PERIOD_MS    (100U)

/*
 * 八路传感器全部位于GPIOB。
 */
#define GRAY_SENSOR_PORT                (GPIOB)

#define GRAY_SENSOR_S1_PIN              (DL_GPIO_PIN_5)
#define GRAY_SENSOR_S2_PIN              (DL_GPIO_PIN_15)
#define GRAY_SENSOR_S3_PIN              (DL_GPIO_PIN_16)
#define GRAY_SENSOR_S4_PIN              (DL_GPIO_PIN_12)
#define GRAY_SENSOR_S5_PIN              (DL_GPIO_PIN_13)
#define GRAY_SENSOR_S6_PIN              (DL_GPIO_PIN_23)
#define GRAY_SENSOR_S7_PIN              (DL_GPIO_PIN_26)
#define GRAY_SENSOR_S8_PIN              (DL_GPIO_PIN_27)

#define GRAY_SENSOR_ALL_PINS            \
    (GRAY_SENSOR_S1_PIN |               \
     GRAY_SENSOR_S2_PIN |               \
     GRAY_SENSOR_S3_PIN |               \
     GRAY_SENSOR_S4_PIN |               \
     GRAY_SENSOR_S5_PIN |               \
     GRAY_SENSOR_S6_PIN |               \
     GRAY_SENSOR_S7_PIN |               \
     GRAY_SENSOR_S8_PIN)

/*
 * 从最左S1到最右S8的位置权重。
 */
static const int16_t g_grayWeights[8] =
{
    3500,
    2500,
    1500,
      500,
     -500,
    -1500,
    -2500,
    -3500
};

/*
 * 模块内部最近一次采样结果。
 */
static GraySensorData_t g_grayLastData;

/*
 * 最近一次串口报告的时间。
 */
static uint32_t g_grayLastReportMs = 0U;


/**
 * @brief 发送一个十六进制半字节。
 */
static void GraySensor_SendHexNibble(uint8_t value)
{
    value &= 0x0FU;

    if (value < 10U)
    {
        Serial_SendByte(
            (uint8_t)('0' + value)
        );
    }
    else
    {
        Serial_SendByte(
            (uint8_t)('A' + value - 10U)
        );
    }
}


/**
 * @brief 发送一个uint8_t的两位十六进制形式。
 */
static void GraySensor_SendHex8(uint8_t value)
{
    GraySensor_SendHexNibble(
        (uint8_t)(value >> 4)
    );

    GraySensor_SendHexNibble(value);
}


/**
 * @brief 发送一个无符号十进制整数。
 */
static void GraySensor_SendUint32(uint32_t value)
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
        buffer[index] =
            (char)('0' + (value % 10U));

        value /= 10U;
        index++;
    }

    while (index > 0U)
    {
        index--;

        Serial_SendByte(
            (uint8_t)buffer[index]
        );
    }
}


/**
 * @brief 发送一个有符号16位十进制整数。
 */
static void GraySensor_SendInt16(int16_t value)
{
    int32_t extendedValue;

    extendedValue = (int32_t)value;

    if (extendedValue < 0)
    {
        Serial_SendByte((uint8_t)'-');
        extendedValue = -extendedValue;
    }

    GraySensor_SendUint32(
        (uint32_t)extendedValue
    );
}


/**
 * @brief 按照车辆物理顺序发送S1到S8。
 *
 * 输出第一个字符对应最左S1，
 * 最后一个字符对应最右S8。
 */
static void GraySensor_SendPhysicalOrder(uint8_t mask)
{
    uint8_t sensorIndex;

    for (sensorIndex = 0U;
         sensorIndex < 8U;
         sensorIndex++)
    {
        if ((mask &
            (uint8_t)(1U << sensorIndex)) != 0U)
        {
            Serial_SendByte((uint8_t)'1');
        }
        else
        {
            Serial_SendByte((uint8_t)'0');
        }
    }
}


/**
 * @brief 发送灰度图案名称。
 */
static void GraySensor_SendPatternName(
    GraySensorPattern_t pattern)
{
    switch (pattern)
    {
        case GRAY_PATTERN_LOST:
        {
            Serial_SendString("LOST");
            break;
        }

        case GRAY_PATTERN_NORMAL:
        {
            Serial_SendString("NORMAL");
            break;
        }

        case GRAY_PATTERN_WIDE:
        {
            Serial_SendString("WIDE");
            break;
        }

        case GRAY_PATTERN_SPLIT:
        {
            Serial_SendString("SPLIT");
            break;
        }

        case GRAY_PATTERN_ALL_BLACK:
        {
            Serial_SendString("ALL_BLACK");
            break;
        }

        default:
        {
            Serial_SendString("UNKNOWN");
            break;
        }
    }
}


/**
 * @brief 统计mask中有效位的数量。
 */
static uint8_t GraySensor_CountActive(uint8_t mask)
{
    uint8_t count = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((mask & (uint8_t)(1U << bit)) != 0U)
        {
            count++;
        }
    }

    return count;
}


/**
 * @brief 统计不连续黑线区域数量。
 *
 * 每次从0变为1时，认为出现一个新的黑线区域。
 */
static uint8_t GraySensor_CountGroups(uint8_t mask)
{
    uint8_t groupCount = 0U;
    uint8_t bit;
    bool previousActive = false;

    for (bit = 0U; bit < 8U; bit++)
    {
        bool currentActive;

        currentActive =
            ((mask & (uint8_t)(1U << bit)) != 0U);

        if (currentActive && !previousActive)
        {
            groupCount++;
        }

        previousActive = currentActive;
    }

    return groupCount;
}


/**
 * @brief 计算黑线加权位置。
 */
static int16_t GraySensor_CalculatePosition(
    uint8_t mask,
    uint8_t activeCount)
{
    int32_t weightSum = 0;
    uint8_t sensorIndex;

    if (activeCount == 0U)
    {
        return 0;
    }

    for (sensorIndex = 0U;
         sensorIndex < 8U;
         sensorIndex++)
    {
        if ((mask &
            (uint8_t)(1U << sensorIndex)) != 0U)
        {
            weightSum +=
                (int32_t)g_grayWeights[sensorIndex];
        }
    }

    return (int16_t)(
        weightSum / (int32_t)activeCount
    );
}


/**
 * @brief 根据mask和统计结果判断图案类型。
 */
static GraySensorPattern_t GraySensor_ClassifyPattern(
    uint8_t mask,
    uint8_t activeCount,
    uint8_t groupCount)
{
    if (activeCount == 0U)
    {
        return GRAY_PATTERN_LOST;
    }

    if (mask == 0xFFU)
    {
        return GRAY_PATTERN_ALL_BLACK;
    }

    if (groupCount > 1U)
    {
        return GRAY_PATTERN_SPLIT;
    }

    if (activeCount >= 4U)
    {
        return GRAY_PATTERN_WIDE;
    }

    return GRAY_PATTERN_NORMAL;
}


/**
 * @brief 通过串口发送完整采样结果。
 */
static void GraySensor_SendStatus(
    const GraySensorData_t *data)
{
    if (data == 0)
    {
        return;
    }

    Serial_SendString("GRAY S1->S8: ");
    GraySensor_SendPhysicalOrder(data->mask);

    Serial_SendString("  mask=0x");
    GraySensor_SendHex8(data->mask);

    Serial_SendString("  count=");
    GraySensor_SendUint32(
        (uint32_t)data->activeCount
    );

    Serial_SendString("  groups=");
    GraySensor_SendUint32(
        (uint32_t)data->groupCount
    );

    Serial_SendString("  pos=");

    if (data->positionValid)
    {
        GraySensor_SendInt16(data->position);
    }
    else
    {
        Serial_SendString("INVALID");
    }

    Serial_SendString("  state=");
    GraySensor_SendPatternName(data->pattern);

    Serial_SendString("\r\n");
}


void GraySensor_Init(void)
{
    g_grayLastReportMs = 0U;
    g_grayLastData = GraySensor_ReadData();

    Serial_SendString("\r\n");
    Serial_SendString("Gray sensor module initialized\r\n");
    Serial_SendString(
        "S1..S8 = PB5 PB15 PB16 PB12 "
        "PB13 PB23 PB26 PB27\r\n"
    );
    Serial_SendString(
        "Weights = -3500 -2500 -1500 -500 "
        "500 1500 2500 3500\r\n"
    );
    Serial_SendString(
        "Raw: Black=0, White=1; "
        "Mask: Black=1, White=0\r\n"
    );

    GraySensor_SendStatus(&g_grayLastData);
}


uint8_t GraySensor_ReadMask(void)
{
    uint32_t portValue;
    uint8_t mask = 0U;

    /*
     * 一次读取八路GPIO。
     */
    portValue = DL_GPIO_readPins(
        GRAY_SENSOR_PORT,
        GRAY_SENSOR_ALL_PINS
    );

    /*
     * 新传感器原始电平：黑线=0，白色=1。
     * 在GPIO读取入口统一反相，使项目内部继续保持：
     * 黑线=1，白色=0。
     *
     * 只保留八路传感器对应的GPIO位，避免按位取反后
     * 其他GPIO位参与后续判断。
     */
    portValue = (~portValue) & GRAY_SENSOR_ALL_PINS;

    if ((portValue & GRAY_SENSOR_S1_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 0);
    }

    if ((portValue & GRAY_SENSOR_S2_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 1);
    }

    if ((portValue & GRAY_SENSOR_S3_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 2);
    }

    if ((portValue & GRAY_SENSOR_S4_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 3);
    }

    if ((portValue & GRAY_SENSOR_S5_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 4);
    }

    if ((portValue & GRAY_SENSOR_S6_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 5);
    }

    if ((portValue & GRAY_SENSOR_S7_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 6);
    }

    if ((portValue & GRAY_SENSOR_S8_PIN) != 0U)
    {
        mask |= (uint8_t)(1U << 7);
    }

    return mask;
}


GraySensorData_t GraySensor_ReadData(void)
{
    GraySensorData_t data;

    data.mask = GraySensor_ReadMask();

    data.activeCount =
        GraySensor_CountActive(data.mask);

    data.groupCount =
        GraySensor_CountGroups(data.mask);

    data.lineDetected =
        (data.activeCount > 0U);

    data.positionValid =
        data.lineDetected;

    data.position =
        GraySensor_CalculatePosition(
            data.mask,
            data.activeCount
        );

    data.pattern =
        GraySensor_ClassifyPattern(
            data.mask,
            data.activeCount,
            data.groupCount
        );

    /*
     * 每次读取GPIO后同步更新最近一次采样缓存。
     *
     * CarControl先调用GraySensor_ReadData()读取当前灰度，
     * LineFollow随后调用GraySensor_GetLastData()时，
     * 即可取得同一帧数据，而不是上电初始化时的旧数据。
     */
    g_grayLastData = data;

    return data;
}


GraySensorData_t GraySensor_GetLastData(void)
{
    return g_grayLastData;
}


void GraySensor_Task(uint32_t nowMs)
{
    if ((uint32_t)(nowMs - g_grayLastReportMs) >=
        GRAY_SENSOR_REPORT_PERIOD_MS)
    {
        g_grayLastReportMs +=
            GRAY_SENSOR_REPORT_PERIOD_MS;

        g_grayLastData = GraySensor_ReadData();

        GraySensor_SendStatus(&g_grayLastData);
    }
}