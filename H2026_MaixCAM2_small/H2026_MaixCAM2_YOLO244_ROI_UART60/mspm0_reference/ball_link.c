#include "ball_link.h"

#include <stdio.h>
#include <string.h>

static uint8_t Checksum8(const uint8_t *data, size_t length)
{
    uint32_t sum = 0U;
    size_t i;
    for (i = 0U; i < length; ++i) sum += data[i];
    return (uint8_t)(sum & 0xFFU);
}

static uint16_t ReadU16LE(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t ReadI16LE(const uint8_t *p)
{
    return (int16_t)ReadU16LE(p);
}

static bool PushFrameByte(uint8_t *buffer,
                          uint8_t *index,
                          uint8_t frame_size,
                          uint8_t second_header,
                          uint8_t byte)
{
    if (*index == 0U)
    {
        if (byte == 0xAAU)
        {
            buffer[0] = byte;
            *index = 1U;
        }
        return false;
    }

    if (*index == 1U)
    {
        if (byte == second_header)
        {
            buffer[1] = byte;
            *index = 2U;
        }
        else if (byte == 0xAAU)
        {
            buffer[0] = byte;
            *index = 1U;
        }
        else
        {
            *index = 0U;
        }
        return false;
    }

    buffer[(*index)++] = byte;
    if (*index < frame_size) return false;
    *index = 0U;

    return Checksum8(buffer, (size_t)frame_size - 1U) ==
           buffer[frame_size - 1U];
}

void BallLink_LegacyInit(BallLinkLegacyParser *parser)
{
    if (parser != NULL) memset(parser, 0, sizeof(*parser));
}

bool BallLink_LegacyPushByte(BallLinkLegacyParser *parser,
                             uint8_t byte,
                             BallLinkLegacyData *out_data)
{
    if (parser == NULL || out_data == NULL) return false;
    if (!PushFrameByte(parser->buffer,
                       &parser->index,
                       BALL_LINK_LEGACY_FRAME_SIZE,
                       0x55U,
                       byte)) return false;

    out_data->sequence = parser->buffer[2];
    out_data->flags = parser->buffer[3];
    out_data->ball_position = ReadU16LE(&parser->buffer[4]);
    out_data->target_position = ReadU16LE(&parser->buffer[6]);
    out_data->score_permille = ReadU16LE(&parser->buffer[8]);
    out_data->mode = parser->buffer[10];
    return true;
}

void BallLink_ExtendedInit(BallLinkExtendedParser *parser)
{
    if (parser != NULL) memset(parser, 0, sizeof(*parser));
}

bool BallLink_ExtendedPushByte(BallLinkExtendedParser *parser,
                               uint8_t byte,
                               BallLinkExtendedData *out_data)
{
    if (parser == NULL || out_data == NULL) return false;
    if (!PushFrameByte(parser->buffer,
                       &parser->index,
                       BALL_LINK_EXTENDED_FRAME_SIZE,
                       0x5AU,
                       byte)) return false;

    out_data->sequence = parser->buffer[2];
    out_data->flags = parser->buffer[3];
    out_data->ball_position = ReadU16LE(&parser->buffer[4]);
    out_data->target_position = ReadU16LE(&parser->buffer[6]);
    out_data->velocity_mm_s = ReadI16LE(&parser->buffer[8]);
    out_data->ball_x_px = ReadU16LE(&parser->buffer[10]);
    out_data->target_x_px = ReadU16LE(&parser->buffer[12]);
    out_data->score_permille = ReadU16LE(&parser->buffer[14]);
    out_data->fps_x10 = ReadU16LE(&parser->buffer[16]);
    out_data->mode = parser->buffer[18];
    return true;
}

void BallLink_Init(BallLinkParser *parser)
{
    BallLink_LegacyInit(parser);
}

bool BallLink_PushByte(BallLinkParser *parser,
                       uint8_t byte,
                       BallLinkData *out_data)
{
    return BallLink_LegacyPushByte(parser, byte, out_data);
}

static size_t CopyLiteral(const char *literal, char *out, size_t out_size)
{
    size_t length;
    if (literal == NULL || out == NULL || out_size == 0U) return 0U;
    length = strlen(literal);
    if (length + 1U > out_size) return 0U;
    memcpy(out, literal, length + 1U);
    return length;
}

size_t BallLink_BuildModeCommand(uint8_t mode, char *out, size_t out_size)
{
    int n;
    if (out == NULL || out_size == 0U) return 0U;
    n = snprintf(out, out_size, "$MODE,%u*", (unsigned int)mode);
    return (n < 0 || (size_t)n >= out_size) ? 0U : (size_t)n;
}

size_t BallLink_BuildTargetNormalizedCommand(uint16_t normalized_position,
                                             char *out,
                                             size_t out_size)
{
    int n;
    if (out == NULL || out_size == 0U) return 0U;
    if (normalized_position > BALL_LINK_POSITION_MAX)
        normalized_position = BALL_LINK_POSITION_MAX;
    n = snprintf(out, out_size, "$TARGETN,%u*",
                 (unsigned int)normalized_position);
    return (n < 0 || (size_t)n >= out_size) ? 0U : (size_t)n;
}

size_t BallLink_BuildTargetPixelCommand(uint16_t pixel_position,
                                        char *out,
                                        size_t out_size)
{
    int n;
    if (out == NULL || out_size == 0U) return 0U;
    n = snprintf(out, out_size, "$TARGET,%u*", (unsigned int)pixel_position);
    return (n < 0 || (size_t)n >= out_size) ? 0U : (size_t)n;
}

size_t BallLink_BuildCaptureCommand(char *out, size_t out_size)
{
    return CopyLiteral("$CAPTURE*", out, out_size);
}

size_t BallLink_BuildCenterCommand(char *out, size_t out_size)
{
    return CopyLiteral("$CENTER*", out, out_size);
}
