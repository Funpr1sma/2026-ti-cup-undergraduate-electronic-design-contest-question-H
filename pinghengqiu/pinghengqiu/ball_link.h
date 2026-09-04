#ifndef BALL_LINK_H_
#define BALL_LINK_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Legacy camera frame, 12 bytes:
 *
 * [0]     0xAA
 * [1]     0x55
 * [2]     sequence
 * [3]     flags
 * [4..5]  ball_position, little-endian
 * [6..7]  target_position, little-endian
 * [8..9]  score_permille, little-endian
 * [10]    mode
 * [11]    checksum
 *
 * checksum = sum(bytes[0..10]) & 0xFF
 */
#define BALL_LINK_LEGACY_FRAME_SIZE       12U
#define BALL_LINK_EXTENDED_FRAME_SIZE     20U
#define BALL_LINK_FRAME_SIZE              BALL_LINK_EXTENDED_FRAME_SIZE

#define BALL_LINK_HEADER_1                0xAAU
#define BALL_LINK_HEADER_2                0x55U
#define BALL_LINK_HEADER_EXTENDED         0x5AU

#define BALL_LINK_INVALID_POSITION        0xFFFFU
#define BALL_LINK_POSITION_MIN            0U
#define BALL_LINK_POSITION_CENTER         5000U
#define BALL_LINK_POSITION_MAX            10000U

#define BALL_LINK_SCORE_MAX               1000U

#define BALL_LINK_FLAG_VALID              (1U << 0)
#define BALL_LINK_FLAG_TARGET_LOCK        (1U << 1)
#define BALL_LINK_FLAG_RTSP_ON            (1U << 2)

/*
 * The current PID safety range is -12..+12 cm.
 * Change this value if the physical distance from board center
 * to either end is not 12 cm.
 */
#define BALL_LINK_BOARD_HALF_RANGE_CM     11.5f

/*
 * +1: camera 0 is left, 10000 is right.
 * -1: camera direction is reversed.
 */
#define BALL_LINK_POSITION_DIRECTION      1

typedef struct
{
    uint8_t sequence;
    uint8_t flags;

    /* 0..10000: left..right, 5000 is center. */
    uint16_t ball_position;

    /* Camera target, not used by the PID controller. */
    uint16_t target_position;

    /* 0..1000. */
    uint16_t score_permille;

    uint8_t mode;
} BallLinkData;

typedef struct
{
    uint8_t buffer[BALL_LINK_FRAME_SIZE];
    uint8_t index;
    uint8_t expected_size;
    uint8_t frame_type; /* 0=none, 1=legacy AA55, 2=extended AA5A */

    uint8_t last_sequence;
    uint8_t has_last_sequence;

    uint32_t valid_frame_count;
    uint32_t legacy_frame_count;
    uint32_t extended_frame_count;
    uint32_t invalid_frame_count;
    uint32_t checksum_error_count;
    uint32_t duplicate_frame_count;

    BallLinkData last_data;
    float last_position_cm;
} BallLinkContext;

/*
 * Initialize the parser and PID bridge.
 * Call once after BallBalance_Init().
 */
void BallLink_Init(void);

/*
 * Feed one received camera UART byte.
 *
 * This function may be called from a UART receive interrupt.
 * It returns 1 only when a new valid ball position has been
 * accepted by BallBalance_PushBallPosition().
 */
uint8_t BallLink_RxByte(uint8_t byte);

/*
 * Convert normalized camera position to PID centimeters.
 *
 * Camera:
 *   0     = left end
 *   5000  = center
 *   10000 = right end
 *
 * PID:
 *   left = negative
 *   center = 0 cm
 *   right = positive
 */
uint8_t BallLink_NormalizedToCm(
    uint16_t normalized_position,
    float *position_cm);

/* Obtain parser state and diagnostics. */
void BallLink_GetContext(BallLinkContext *context);

/* Reset only communication diagnostics and parser state. */
void BallLink_Reset(void);

/*
 * Optional camera command builders.
 * Return command length, or 0 if output buffer is too small.
 */
size_t BallLink_BuildModeCommand(
    uint8_t mode,
    char *out,
    size_t out_size);

size_t BallLink_BuildTargetNormalizedCommand(
    uint16_t normalized_position,
    char *out,
    size_t out_size);

size_t BallLink_BuildTargetPixelCommand(
    uint16_t pixel_position,
    char *out,
    size_t out_size);

size_t BallLink_BuildCaptureCommand(
    char *out,
    size_t out_size);

size_t BallLink_BuildCenterCommand(
    char *out,
    size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* BALL_LINK_H_ */