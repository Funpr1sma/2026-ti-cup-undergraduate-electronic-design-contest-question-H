#include "ball_link.h"

#include <stdio.h>
#include <string.h>

#include "ball_balance_control.h"
#include "clock.h"
#include "ti_msp_dl_config.h"

static BallLinkContext s_ball_link;

static uint8_t BallLink_Checksum8(
    const uint8_t *data,
    size_t length)
{
    uint32_t sum;
    size_t index;

    sum = 0U;

    for (index = 0U; index < length; index++) {
        sum += data[index];
    }

    return (uint8_t)(sum & 0xFFU);
}

static uint16_t BallLink_ReadU16LE(
    const uint8_t *data)
{
    return (uint16_t)data[0] |
           ((uint16_t)data[1] << 8);
}

static void BallLink_DecodeFrame(
    const uint8_t *frame,
    uint8_t frame_type,
    BallLinkData *data)
{
    data->sequence = frame[2];
    data->flags = frame[3];
    data->ball_position = BallLink_ReadU16LE(&frame[4]);
    data->target_position = BallLink_ReadU16LE(&frame[6]);

    if (frame_type == 2U) {
        /* Extended AA 5A frame: score is at bytes 14..15, mode at 18. */
        data->score_permille = BallLink_ReadU16LE(&frame[14]);
        data->mode = frame[18];
    } else {
        /* Legacy AA 55 frame: score is at bytes 8..9, mode at 10. */
        data->score_permille = BallLink_ReadU16LE(&frame[8]);
        data->mode = frame[10];
    }
}

/*
 * Return values:
 *   0 = no complete frame
 *   1 = complete legacy frame with correct checksum
 *   2 = complete frame with checksum error
 *   3 = complete extended frame with correct checksum
 */
static uint8_t BallLink_ParseByte(
    uint8_t byte,
    BallLinkData *data)
{
    uint8_t expected_checksum;
    uint8_t frame_type;

    if (s_ball_link.index == 0U) {
        if (byte == BALL_LINK_HEADER_1) {
            s_ball_link.buffer[0] = byte;
            s_ball_link.index = 1U;
        }
        return 0U;
    }

    if (s_ball_link.index == 1U) {
        if (byte == BALL_LINK_HEADER_2) {
            s_ball_link.buffer[1] = byte;
            s_ball_link.index = 2U;
            s_ball_link.expected_size = BALL_LINK_LEGACY_FRAME_SIZE;
            s_ball_link.frame_type = 1U;
        } else if (byte == BALL_LINK_HEADER_EXTENDED) {
            s_ball_link.buffer[1] = byte;
            s_ball_link.index = 2U;
            s_ball_link.expected_size = BALL_LINK_EXTENDED_FRAME_SIZE;
            s_ball_link.frame_type = 2U;
        } else if (byte == BALL_LINK_HEADER_1) {
            s_ball_link.buffer[0] = byte;
            s_ball_link.index = 1U;
        } else {
            s_ball_link.index = 0U;
            s_ball_link.expected_size = 0U;
            s_ball_link.frame_type = 0U;
        }
        return 0U;
    }

    if (s_ball_link.expected_size == 0U ||
        s_ball_link.expected_size > BALL_LINK_FRAME_SIZE) {
        s_ball_link.index = 0U;
        s_ball_link.frame_type = 0U;
        return 0U;
    }

    s_ball_link.buffer[s_ball_link.index] = byte;
    s_ball_link.index++;

    if (s_ball_link.index < s_ball_link.expected_size) {
        return 0U;
    }

    frame_type = s_ball_link.frame_type;
    s_ball_link.index = 0U;
    s_ball_link.frame_type = 0U;

    expected_checksum = BallLink_Checksum8(
        s_ball_link.buffer,
        (size_t)s_ball_link.expected_size - 1U);

    if (expected_checksum !=
        s_ball_link.buffer[s_ball_link.expected_size - 1U]) {
        s_ball_link.expected_size = 0U;
        return 2U;
    }

    BallLink_DecodeFrame(s_ball_link.buffer, frame_type, data);
    s_ball_link.expected_size = 0U;

    return (frame_type == 2U) ? 3U : 1U;
}

uint8_t BallLink_NormalizedToCm(
    uint16_t normalized_position,
    float *position_cm)
{
    int32_t centered_position;
    float converted_position;

    if (position_cm == 0) {
        return 0U;
    }

    if (normalized_position ==
            BALL_LINK_INVALID_POSITION ||
        normalized_position >
            BALL_LINK_POSITION_MAX) {
        return 0U;
    }

    centered_position =
        (int32_t)normalized_position -
        (int32_t)BALL_LINK_POSITION_CENTER;

    converted_position =
        ((float)centered_position *
         BALL_LINK_BOARD_HALF_RANGE_CM) /
        (float)BALL_LINK_POSITION_CENTER;

#if BALL_LINK_POSITION_DIRECTION < 0
    converted_position = -converted_position;
#endif

    *position_cm = converted_position;

    return 1U;
}

void BallLink_Reset(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    memset(&s_ball_link, 0, sizeof(s_ball_link));

    if (primask == 0U) {
        __enable_irq();
    }
}

void BallLink_Init(void)
{
    BallLink_Reset();
}

uint8_t BallLink_RxByte(uint8_t byte)
{
    BallLinkData data;
    float position_cm;
    uint8_t parse_result;

    parse_result = BallLink_ParseByte(
        byte,
        &data);

    if (parse_result == 0U) {
        return 0U;
    }

    if (parse_result == 2U) {
        s_ball_link.checksum_error_count++;
        s_ball_link.invalid_frame_count++;
        return 0U;
    }

    if (parse_result == 3U) {
        s_ball_link.extended_frame_count++;
    } else {
        s_ball_link.legacy_frame_count++;
    }

    /*
     * A repeated sequence is probably a retransmitted frame.
     * Feeding it to the PID would create a false new measurement.
     */
    if (s_ball_link.has_last_sequence != 0U &&
        data.sequence == s_ball_link.last_sequence) {
        s_ball_link.duplicate_frame_count++;
        return 0U;
    }

    /*
     * Record every correctly decoded frame sequence, including
     * invalid detection frames.
     */
    s_ball_link.last_sequence = data.sequence;
    s_ball_link.has_last_sequence = 1U;
    s_ball_link.last_data = data;

    /*
     * No ball was detected. Do not inject zero or an old position.
     * The controller will raise a configured camera timeout without a
     * valid call to BallBalance_PushBallPosition().
     */
    if ((data.flags & BALL_LINK_FLAG_VALID) == 0U) {
        s_ball_link.invalid_frame_count++;
        return 0U;
    }

    if (BallLink_NormalizedToCm(
            data.ball_position,
            &position_cm) == 0U) {
        s_ball_link.invalid_frame_count++;
        return 0U;
    }

    /*
     * score_permille is retained for diagnostics. A score
     * threshold can be added after the camera score is calibrated.
     */
    if (data.score_permille > BALL_LINK_SCORE_MAX) {
        s_ball_link.invalid_frame_count++;
        return 0U;
    }

    if (BallBalance_PushBallPosition(
            position_cm,
            (uint32_t)tick_ms) == 0U) {
        s_ball_link.invalid_frame_count++;
        return 0U;
    }

    s_ball_link.last_position_cm = position_cm;
    s_ball_link.valid_frame_count++;

    return 1U;
}

void BallLink_GetContext(
    BallLinkContext *context)
{
    uint32_t primask;

    if (context == 0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *context = s_ball_link;

    if (primask == 0U) {
        __enable_irq();
    }
}

static size_t BallLink_CopyLiteral(
    const char *literal,
    char *out,
    size_t out_size)
{
    size_t length;

    if (literal == 0 ||
        out == 0 ||
        out_size == 0U) {
        return 0U;
    }

    length = strlen(literal);

    if (length + 1U > out_size) {
        return 0U;
    }

    memcpy(out, literal, length + 1U);

    return length;
}

size_t BallLink_BuildModeCommand(
    uint8_t mode,
    char *out,
    size_t out_size)
{
    int length;

    if (out == 0 || out_size == 0U) {
        return 0U;
    }

    length = snprintf(
        out,
        out_size,
        "$MODE,%u*",
        (unsigned int)mode);

    if (length < 0 ||
        (size_t)length >= out_size) {
        return 0U;
    }

    return (size_t)length;
}

size_t BallLink_BuildTargetNormalizedCommand(
    uint16_t normalized_position,
    char *out,
    size_t out_size)
{
    int length;

    if (out == 0 || out_size == 0U) {
        return 0U;
    }

    if (normalized_position >
        BALL_LINK_POSITION_MAX) {
        normalized_position =
            BALL_LINK_POSITION_MAX;
    }

    length = snprintf(
        out,
        out_size,
        "$TARGETN,%u*",
        (unsigned int)normalized_position);

    if (length < 0 ||
        (size_t)length >= out_size) {
        return 0U;
    }

    return (size_t)length;
}

size_t BallLink_BuildTargetPixelCommand(
    uint16_t pixel_position,
    char *out,
    size_t out_size)
{
    int length;

    if (out == 0 || out_size == 0U) {
        return 0U;
    }

    length = snprintf(
        out,
        out_size,
        "$TARGET,%u*",
        (unsigned int)pixel_position);

    if (length < 0 ||
        (size_t)length >= out_size) {
        return 0U;
    }

    return (size_t)length;
}

size_t BallLink_BuildCaptureCommand(
    char *out,
    size_t out_size)
{
    return BallLink_CopyLiteral(
        "$CAPTURE*",
        out,
        out_size);
}

size_t BallLink_BuildCenterCommand(
    char *out,
    size_t out_size)
{
    return BallLink_CopyLiteral(
        "$CENTER*",
        out,
        out_size);
}
