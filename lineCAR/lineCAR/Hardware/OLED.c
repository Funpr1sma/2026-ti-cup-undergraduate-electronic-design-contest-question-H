#include "OLED.h"

#include "CarControl.h"
#include "Config/CarConfig.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#if CAR_ENABLE_OLED != 0U

#define OLED_CONTROL_COMMAND          (0x00U)
#define OLED_CONTROL_DATA             (0x40U)

/*
 * MSPM0 I2C控制器TX FIFO最多容纳8字节。
 *
 * 第一个字节用于SSD1306控制字，因此每包最多发送7字节显示数据。
 */
#define OLED_I2C_PACKET_SIZE          (8U)
#define OLED_DATA_CHUNK_SIZE          (7U)

/*
 * 原有大号时间显示参数。
 */
#define OLED_TIME_CHAR_COUNT          (8U)
#define OLED_SOURCE_CHAR_WIDTH        (6U)
#define OLED_SCALED_CHAR_WIDTH        (12U)

#define OLED_TIME_FIRST_COLUMN        \
    ((OLED_WIDTH -                    \
      (OLED_TIME_CHAR_COUNT *         \
       OLED_SCALED_CHAR_WIDTH)) / 2U)

/*
 * 5x7小字体参数。
 *
 * 每个字符占5列字形和1列间隔。
 * 128像素宽度最多显示21个字符。
 */
#define OLED_TEXT_CHAR_WIDTH          (6U)
#define OLED_TEXT_MAX_CHARS           \
    (OLED_WIDTH / OLED_TEXT_CHAR_WIDTH)

#if OLED_HEIGHT == 64U

#define OLED_TIME_FIRST_PAGE          (3U)
#define OLED_MULTIPLEX_VALUE          (0x3FU)
#define OLED_COM_PIN_CONFIG           (0x12U)

#elif OLED_HEIGHT == 32U

#define OLED_TIME_FIRST_PAGE          (1U)
#define OLED_MULTIPLEX_VALUE          (0x1FU)
#define OLED_COM_PIN_CONFIG           (0x02U)

#else

#error "OLED_HEIGHT must be 32 or 64"

#endif

typedef enum
{
    /*
     * 原来的车辆计时显示模式。
     */
    OLED_MODE_TIME = 0,

    /*
     * 独立测试和编码器标定使用的文本模式。
     */
    OLED_MODE_TEXT

} OLEDDisplayMode_t;


static bool g_oledOnline;
static uint32_t g_lastTaskMs;
static uint32_t g_lastRetryMs;
static uint32_t g_i2cStartDelayCycles;
static uint32_t g_connectAttempts;
static uint32_t g_connectFailures;
static uint32_t g_runtimeFailures;

static OLEDDisplayMode_t g_displayMode;

static char g_displayedText[OLED_TIME_CHAR_COUNT];
static char g_targetText[OLED_TIME_CHAR_COUNT];


/*
 * 5x7数字字库。
 *
 * 每个字节表示一列垂直像素。
 */
static const uint8_t g_digitFont[10][5] =
{
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}, /* 0 */
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}, /* 1 */
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}, /* 2 */
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}, /* 3 */
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}, /* 4 */
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}, /* 5 */
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}, /* 6 */
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}, /* 7 */
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}, /* 8 */
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}  /* 9 */
};


/*
 * 5x7大写英文字库。
 */
static const uint8_t g_upperFont[26][5] =
{
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}, /* A */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}, /* B */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}, /* C */
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}, /* D */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}, /* E */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}, /* F */
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}, /* G */
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}, /* H */
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}, /* I */
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}, /* J */
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}, /* K */
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}, /* L */
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}, /* M */
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}, /* N */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}, /* O */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}, /* P */
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}, /* Q */
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}, /* R */
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}, /* S */
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}, /* T */
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}, /* U */
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}, /* V */
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}, /* W */
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}, /* X */
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}, /* Y */
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}  /* Z */
};


static const uint8_t g_colonFont[5] =
{
    0x00U, 0x36U, 0x36U, 0x00U, 0x00U
};


static const uint8_t g_minusFont[5] =
{
    0x08U, 0x08U, 0x08U, 0x08U, 0x08U
};


static const uint8_t g_dotFont[5] =
{
    0x00U, 0x60U, 0x60U, 0x00U, 0x00U
};


static const uint8_t g_slashFont[5] =
{
    0x20U, 0x10U, 0x08U, 0x04U, 0x02U
};


static const uint8_t g_equalFont[5] =
{
    0x14U, 0x14U, 0x14U, 0x14U, 0x14U
};


static const uint8_t g_percentFont[5] =
{
    0x63U, 0x13U, 0x08U, 0x64U, 0x63U
};


static const uint8_t g_plusFont[5] =
{
    0x08U, 0x08U, 0x3EU, 0x08U, 0x08U
};


static const uint8_t g_blankFont[5] =
{
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};


static bool OLED_TaskDue(
    uint32_t nowMs,
    uint32_t *lastMs,
    uint32_t periodMs)
{
    if ((uint32_t)(nowMs - *lastMs) < periodMs)
    {
        return false;
    }

    *lastMs += periodMs;

    if ((uint32_t)(nowMs - *lastMs) >= periodMs)
    {
        *lastMs = nowMs;
    }

    return true;
}


static void OLED_InvalidateTimeCache(void)
{
    uint8_t index;

    for (index = 0U;
         index < OLED_TIME_CHAR_COUNT;
         index++)
    {
        g_displayedText[index] = '\0';
        g_targetText[index] = '\0';
    }
}


static void OLED_RecoverI2C(void)
{
    DL_I2C_flushControllerTXFIFO(
        OLED_I2C_INST
    );

    DL_I2C_resetControllerTransfer(
        OLED_I2C_INST
    );
}


static bool OLED_WaitForIdle(void)
{
    uint32_t timeout =
        OLED_I2C_TIMEOUT_LOOPS;

    while ((DL_I2C_getControllerStatus(
                OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    {
        if (timeout == 0U)
        {
            OLED_RecoverI2C();
            return false;
        }

        timeout--;
    }

    return true;
}


static bool OLED_WaitForTransferComplete(void)
{
    uint32_t timeout =
        OLED_I2C_TIMEOUT_LOOPS;

    uint32_t status;

    do
    {
        status =
            DL_I2C_getControllerStatus(
                OLED_I2C_INST
            );

        if (timeout == 0U)
        {
            OLED_RecoverI2C();
            return false;
        }

        timeout--;
    }
    while ((status &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U);

    if ((status &
         DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
    {
        OLED_RecoverI2C();
        return false;
    }

    return true;
}


static bool OLED_I2C_Transmit(
    const uint8_t *data,
    uint8_t length)
{
    uint16_t loaded;

    if ((data == 0) ||
        (length == 0U) ||
        (length > OLED_I2C_PACKET_SIZE))
    {
        return false;
    }

    if (!OLED_WaitForIdle())
    {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(
        OLED_I2C_INST
    );

    loaded =
        DL_I2C_fillControllerTXFIFO(
            OLED_I2C_INST,
            data,
            length
        );

    if (loaded != length)
    {
        OLED_RecoverI2C();
        return false;
    }

    DL_I2C_startControllerTransfer(
        OLED_I2C_INST,
        OLED_I2C_ADDRESS_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        length
    );

    /*
     * TI针对I2C_ERR_13建议的启动延迟。
     */
    delay_cycles(g_i2cStartDelayCycles);

    return OLED_WaitForTransferComplete();
}


static bool OLED_WriteCommand(uint8_t command)
{
    uint8_t packet[2];

    packet[0] = OLED_CONTROL_COMMAND;
    packet[1] = command;

    return OLED_I2C_Transmit(
        packet,
        2U
    );
}


static bool OLED_WriteData(
    const uint8_t *data,
    uint16_t length)
{
    uint8_t packet[OLED_I2C_PACKET_SIZE];

    while (length > 0U)
    {
        uint8_t chunk;
        uint8_t index;

        chunk =
            (length > OLED_DATA_CHUNK_SIZE) ?
            OLED_DATA_CHUNK_SIZE :
            (uint8_t)length;

        packet[0] = OLED_CONTROL_DATA;

        for (index = 0U;
             index < chunk;
             index++)
        {
            packet[index + 1U] = data[index];
        }

        if (!OLED_I2C_Transmit(
                packet,
                (uint8_t)(chunk + 1U)))
        {
            return false;
        }

        data += chunk;
        length -= chunk;
    }

    return true;
}


static bool OLED_SetPosition(
    uint8_t page,
    uint8_t column)
{
    uint8_t physicalColumn;

    if ((page >= OLED_PAGE_COUNT) ||
        (column >= OLED_WIDTH))
    {
        return false;
    }

    physicalColumn =
        (uint8_t)(
            column + OLED_COLUMN_OFFSET
        );

    return
        OLED_WriteCommand(
            (uint8_t)(0xB0U | page)
        ) &&
        OLED_WriteCommand(
            (uint8_t)(
                physicalColumn & 0x0FU
            )
        ) &&
        OLED_WriteCommand(
            (uint8_t)(
                0x10U |
                (physicalColumn >> 4U)
            )
        );
}


static void OLED_ConfigureStartDelay(void)
{
    DL_I2C_ClockConfig clockConfig;

    uint32_t sourceHz = 4000000U;
    uint32_t cpuCyclesPerI2cClock;

    DL_I2C_getClockConfig(
        OLED_I2C_INST,
        &clockConfig
    );

    if (clockConfig.clockSel ==
        DL_I2C_CLOCK_BUSCLK)
    {
        sourceHz = CPUCLK_FREQ;
    }
    else if (clockConfig.clockSel ==
             DL_I2C_CLOCK_MFCLK)
    {
        sourceHz = 4000000U;
    }

    cpuCyclesPerI2cClock =
        CPUCLK_FREQ / sourceHz;

    if (cpuCyclesPerI2cClock == 0U)
    {
        cpuCyclesPerI2cClock = 1U;
    }

    g_i2cStartDelayCycles =
        3U *
        ((uint32_t)clockConfig.divideRatio + 1U) *
        cpuCyclesPerI2cClock;

    if (g_i2cStartDelayCycles == 0U)
    {
        g_i2cStartDelayCycles = 1U;
    }
}


static bool OLED_ControllerConfigure(void)
{
    static const uint8_t commands[] =
    {
        0xAEU,                       /* display off */
        0xD5U, 0x80U,                /* display clock */
        0xA8U, OLED_MULTIPLEX_VALUE, /* multiplex */
        0xD3U, 0x00U,                /* display offset */
        0x40U,                       /* display start line */
        0x8DU, 0x14U,                /* charge pump */
        0x20U, 0x02U,                /* page addressing */
        0xA1U,                       /* segment remap */
        0xC8U,                       /* COM scan direction */
        0xDAU, OLED_COM_PIN_CONFIG,  /* COM pin config */
        0x81U, 0x7FU,                /* contrast */
        0xD9U, 0xF1U,                /* pre-charge */
        0xDBU, 0x40U,                /* VCOM detect */
        0xA4U,                       /* use display RAM */
        0xA6U,                       /* normal display */
        0x2EU                        /* stop scrolling */
    };

    uint8_t index;

    for (index = 0U;
         index < (uint8_t)sizeof(commands);
         index++)
    {
        if (!OLED_WriteCommand(commands[index]))
        {
            return false;
        }
    }

    return true;
}


static bool OLED_ClearHardware(void)
{
    static const uint8_t zeros[
        OLED_DATA_CHUNK_SIZE
    ] =
    {
        0U, 0U, 0U, 0U, 0U, 0U, 0U
    };

    uint8_t page;

    for (page = 0U;
         page < OLED_PAGE_COUNT;
         page++)
    {
        uint16_t remaining = OLED_WIDTH;

        if (!OLED_SetPosition(page, 0U))
        {
            return false;
        }

        while (remaining > 0U)
        {
            uint16_t chunk =
                (remaining >
                 OLED_DATA_CHUNK_SIZE) ?
                OLED_DATA_CHUNK_SIZE :
                remaining;

            if (!OLED_WriteData(
                    zeros,
                    chunk))
            {
                return false;
            }

            remaining -= chunk;
        }
    }

    return true;
}


static const uint8_t *OLED_GetGlyph(
    char character)
{
    if ((character >= '0') &&
        (character <= '9'))
    {
        return g_digitFont[
            (uint8_t)(character - '0')
        ];
    }

    if ((character >= 'a') &&
        (character <= 'z'))
    {
        character =
            (char)(
                character - 'a' + 'A'
            );
    }

    if ((character >= 'A') &&
        (character <= 'Z'))
    {
        return g_upperFont[
            (uint8_t)(character - 'A')
        ];
    }

    switch (character)
    {
        case ':':
            return g_colonFont;

        case '-':
            return g_minusFont;

        case '.':
            return g_dotFont;

        case '/':
            return g_slashFont;

        case '=':
            return g_equalFont;

        case '%':
            return g_percentFont;

        case '+':
            return g_plusFont;

        case ' ':
        default:
            return g_blankFont;
    }
}


static bool OLED_DrawLargeCharacter(
    uint8_t characterIndex,
    char character)
{
    const uint8_t *glyph =
        OLED_GetGlyph(character);

    uint8_t upper[OLED_SCALED_CHAR_WIDTH];
    uint8_t lower[OLED_SCALED_CHAR_WIDTH];

    uint8_t sourceColumn;
    uint8_t outputColumn = 0U;
    uint8_t column;

    for (sourceColumn = 0U;
         sourceColumn < OLED_SOURCE_CHAR_WIDTH;
         sourceColumn++)
    {
        uint8_t sourceBits =
            (sourceColumn < 5U) ?
            glyph[sourceColumn] :
            0U;

        uint16_t expandedBits = 0U;
        uint8_t bit;

        for (bit = 0U;
             bit < 8U;
             bit++)
        {
            if ((sourceBits &
                 (uint8_t)(1U << bit)) != 0U)
            {
                expandedBits |=
                    (uint16_t)(
                        3U << (bit * 2U)
                    );
            }
        }

        upper[outputColumn] =
            (uint8_t)(
                expandedBits & 0xFFU
            );

        lower[outputColumn] =
            (uint8_t)(
                expandedBits >> 8U
            );

        outputColumn++;

        upper[outputColumn] =
            (uint8_t)(
                expandedBits & 0xFFU
            );

        lower[outputColumn] =
            (uint8_t)(
                expandedBits >> 8U
            );

        outputColumn++;
    }

    column =
        (uint8_t)(
            OLED_TIME_FIRST_COLUMN +
            characterIndex *
            OLED_SCALED_CHAR_WIDTH
        );

    if (!OLED_SetPosition(
            OLED_TIME_FIRST_PAGE,
            column) ||
        !OLED_WriteData(
            upper,
            OLED_SCALED_CHAR_WIDTH) ||
        !OLED_SetPosition(
            (uint8_t)(
                OLED_TIME_FIRST_PAGE + 1U
            ),
            column))
    {
        return false;
    }

    return OLED_WriteData(
        lower,
        OLED_SCALED_CHAR_WIDTH
    );
}


static bool OLED_DrawSmallCharacter(
    uint8_t line,
    uint8_t characterIndex,
    char character)
{
    const uint8_t *glyph;
    uint8_t columns[OLED_TEXT_CHAR_WIDTH];
    uint8_t index;
    uint8_t column;

    if ((line >= OLED_PAGE_COUNT) ||
        (characterIndex >=
         OLED_TEXT_MAX_CHARS))
    {
        return false;
    }

    glyph = OLED_GetGlyph(character);

    for (index = 0U;
         index < 5U;
         index++)
    {
        columns[index] = glyph[index];
    }

    /*
     * 字符之间留一列空白。
     */
    columns[5] = 0U;

    column =
        (uint8_t)(
            characterIndex *
            OLED_TEXT_CHAR_WIDTH
        );

    return
        OLED_SetPosition(line, column) &&
        OLED_WriteData(
            columns,
            OLED_TEXT_CHAR_WIDTH
        );
}


static void OLED_FormatTime(
    uint32_t totalSeconds,
    char output[OLED_TIME_CHAR_COUNT])
{
    uint32_t hours =
        totalSeconds / 3600U;

    uint32_t minutes;
    uint32_t seconds;

    if (hours > 99U)
    {
        hours = 99U;
    }

    minutes =
        (totalSeconds / 60U) % 60U;

    seconds =
        totalSeconds % 60U;

    output[0] =
        (char)('0' + (hours / 10U));

    output[1] =
        (char)('0' + (hours % 10U));

    output[2] = ':';

    output[3] =
        (char)('0' + (minutes / 10U));

    output[4] =
        (char)('0' + (minutes % 10U));

    output[5] = ':';

    output[6] =
        (char)('0' + (seconds / 10U));

    output[7] =
        (char)('0' + (seconds % 10U));
}


static bool OLED_DrawFullTime(void)
{
    uint8_t index;

    OLED_FormatTime(
        CarControl_GetDriveTimeMs() / 1000U,
        g_targetText
    );

    for (index = 0U;
         index < OLED_TIME_CHAR_COUNT;
         index++)
    {
        if (!OLED_DrawLargeCharacter(
                index,
                g_targetText[index]))
        {
            return false;
        }

        g_displayedText[index] =
            g_targetText[index];
    }

    return true;
}


static bool OLED_Connect(void)
{
    bool success;

    g_connectAttempts++;
    OLED_RecoverI2C();

    success = OLED_ControllerConfigure() &&
        OLED_ClearHardware() &&
        OLED_WriteCommand(0xAFU);

    if (success)
    {
        OLED_InvalidateTimeCache();
        g_displayMode = OLED_MODE_TIME;
        success = OLED_DrawFullTime();
    }

    if (!success)
    {
        g_connectFailures++;
        OLED_RecoverI2C();
    }

    return success;
}


void OLED_Init(uint32_t nowMs)
{
    uint8_t attempt;

    g_oledOnline = false;
    g_lastTaskMs = nowMs;
    g_lastRetryMs = nowMs;
    g_displayMode = OLED_MODE_TIME;
    g_connectAttempts = 0U;
    g_connectFailures = 0U;
    g_runtimeFailures = 0U;

    OLED_InvalidateTimeCache();
    OLED_ConfigureStartDelay();

    /*
     * A cold power-up can leave the SSD1306 charge pump or I2C interface not
     * ready while the MCU has already started. Reset works because the OLED is
     * then fully powered. Wait longer and retry before normal operation.
     */
    delay_cycles(
        (uint32_t)(
            ((uint64_t)CPUCLK_FREQ * OLED_POWER_ON_DELAY_MS) / 1000ULL));

    for (attempt = 0U;
         attempt < OLED_BOOT_RETRY_COUNT;
         attempt++)
    {
        if (OLED_Connect())
        {
            g_oledOnline = true;
            break;
        }

        if ((uint8_t)(attempt + 1U) < OLED_BOOT_RETRY_COUNT)
        {
            delay_cycles(
                (uint32_t)(
                    ((uint64_t)CPUCLK_FREQ * OLED_BOOT_RETRY_GAP_MS) /
                    1000ULL));
        }
    }

    g_lastRetryMs = nowMs;
}


void OLED_Task(uint32_t nowMs)
{
    uint8_t index;

    if (!OLED_TaskDue(
            nowMs,
            &g_lastTaskMs,
            OLED_TASK_PERIOD_MS))
    {
        return;
    }

    /*
     * 文本模式下绝不使用计时内容覆盖屏幕。
     *
     * 编码器标定主函数即使误调用OLED_Task，
     * 标定信息仍然会保留。
     */
    if (g_displayMode == OLED_MODE_TEXT)
    {
        return;
    }

    if (!g_oledOnline)
    {
        /*
         * 车辆正在运行时不执行可能产生较长超时的重连。
         */
        /*
         * Reconnect is allowed while idle or waiting on the start marker /
         * ARMING, because the wheels are still stopped. Avoid a possibly
         * lengthy I2C recovery only after motor control has entered an active
         * drive or stopping state.
         */
        {
            CarControlStatus_t carStatus = CarControl_GetStatus();

            if ((carStatus.state == CAR_STATE_PRETILT) ||
                (carStatus.state == CAR_STATE_RUNNING) ||
                (carStatus.state == CAR_STATE_PASSING_FINISH) ||
                (carStatus.state == CAR_STATE_STOPPING))
            {
                return;
            }
        }

        if ((uint32_t)(
                nowMs - g_lastRetryMs
            ) < OLED_RETRY_PERIOD_MS)
        {
            return;
        }

        g_lastRetryMs = nowMs;
        g_oledOnline = OLED_Connect();

        return;
    }

    OLED_FormatTime(
        CarControl_GetDriveTimeMs() / 1000U,
        g_targetText
    );

    /*
     * 每次任务最多更新一个发生变化的字符，
     * 减少I2C操作对控制循环的影响。
     */
    for (index = 0U;
         index < OLED_TIME_CHAR_COUNT;
         index++)
    {
        if (g_targetText[index] !=
            g_displayedText[index])
        {
            if (!OLED_DrawLargeCharacter(
                    index,
                    g_targetText[index]))
            {
                g_oledOnline = false;
                g_runtimeFailures++;
                g_lastRetryMs = nowMs;
                return;
            }

            g_displayedText[index] =
                g_targetText[index];

            return;
        }
    }
}


bool OLED_IsOnline(void)
{
    return g_oledOnline;
}


void OLED_GetDiagnostics(OLEDDiagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return;
    }

    diagnostics->connectAttempts = g_connectAttempts;
    diagnostics->connectFailures = g_connectFailures;
    diagnostics->runtimeFailures = g_runtimeFailures;
    diagnostics->online = g_oledOnline ? 1U : 0U;
}


bool OLED_ForceReconnect(uint32_t nowMs)
{
    g_oledOnline = false;
    g_lastRetryMs = nowMs;
    OLED_InvalidateTimeCache();
    g_oledOnline = OLED_Connect();
    return g_oledOnline;
}


bool OLED_ClearScreen(void)
{
    if (!g_oledOnline)
    {
        return false;
    }

    g_displayMode = OLED_MODE_TEXT;

    if (!OLED_ClearHardware())
    {
        g_oledOnline = false;
        g_runtimeFailures++;
        return false;
    }

    OLED_InvalidateTimeCache();
    return true;
}


bool OLED_DrawText(
    uint8_t line,
    const char *text)
{
    uint8_t index;
    bool endReached = false;

    if ((!g_oledOnline) ||
        (text == 0) ||
        (line >= OLED_PAGE_COUNT))
    {
        return false;
    }

    g_displayMode = OLED_MODE_TEXT;

    for (index = 0U;
         index < OLED_TEXT_MAX_CHARS;
         index++)
    {
        char character;

        if (!endReached && (*text != '\0'))
        {
            character = *text;
            text++;
        }
        else
        {
            character = ' ';
            endReached = true;
        }

        if (!OLED_DrawSmallCharacter(
                line,
                index,
                character))
        {
            g_oledOnline = false;
            g_runtimeFailures++;
            return false;
        }
    }

    return true;
}


bool OLED_ResumeTimeDisplay(void)
{
    if (!g_oledOnline)
    {
        return false;
    }

    g_displayMode = OLED_MODE_TIME;
    OLED_InvalidateTimeCache();

    if (!OLED_ClearHardware())
    {
        g_oledOnline = false;
        g_runtimeFailures++;
        return false;
    }

    return OLED_DrawFullTime();
}


#else

/*
 * CAR_ENABLE_OLED为0时保留所有空接口，
 * 使正常程序和标定程序都可以正常编译。
 */

void OLED_Init(uint32_t nowMs)
{
    (void)nowMs;
}


void OLED_Task(uint32_t nowMs)
{
    (void)nowMs;
}


bool OLED_IsOnline(void)
{
    return false;
}


void OLED_GetDiagnostics(OLEDDiagnostics_t *diagnostics)
{
    if (diagnostics != 0)
    {
        diagnostics->connectAttempts = 0U;
        diagnostics->connectFailures = 0U;
        diagnostics->runtimeFailures = 0U;
        diagnostics->online = 0U;
    }
}


bool OLED_ForceReconnect(uint32_t nowMs)
{
    (void)nowMs;
    return false;
}


bool OLED_ClearScreen(void)
{
    return false;
}


bool OLED_DrawText(
    uint8_t line,
    const char *text)
{
    (void)line;
    (void)text;

    return false;
}


bool OLED_ResumeTimeDisplay(void)
{
    return false;
}

#endif /* CAR_ENABLE_OLED */