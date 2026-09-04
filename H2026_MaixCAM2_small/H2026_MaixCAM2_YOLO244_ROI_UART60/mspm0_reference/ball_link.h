#ifndef BALL_LINK_H_
#define BALL_LINK_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Original frame: AA 55 ... checksum, kept for compatibility. */
#define BALL_LINK_LEGACY_FRAME_SIZE       12U
/* Extended frame: AA 5A ... checksum. */
#define BALL_LINK_EXTENDED_FRAME_SIZE     20U

#define BALL_LINK_INVALID_POSITION        0xFFFFU
#define BALL_LINK_INVALID_PIXEL           0xFFFFU
#define BALL_LINK_POSITION_CENTER         5000U
#define BALL_LINK_POSITION_MAX            10000U

#define BALL_LINK_FLAG_VALID              (1U << 0)
#define BALL_LINK_FLAG_TARGET_LOCK        (1U << 1)
#define BALL_LINK_FLAG_RTSP_ON            (1U << 2)
#define BALL_LINK_FLAG_VELOCITY_VALID     (1U << 3)
#define BALL_LINK_FLAG_EXTENDED_ENABLED   (1U << 4)

typedef struct
{
    uint8_t sequence;
    uint8_t flags;
    uint16_t ball_position;
    uint16_t target_position;
    uint16_t score_permille;
    uint8_t mode;
} BallLinkLegacyData;

typedef struct
{
    uint8_t sequence;
    uint8_t flags;
    uint16_t ball_position;      /* 0..10000, left..right */
    uint16_t target_position;    /* 0..10000 */
    int16_t velocity_mm_s;       /* signed ball velocity */
    uint16_t ball_x_px;          /* 480x272 preview coordinate */
    uint16_t target_x_px;        /* 480x272 preview coordinate */
    uint16_t score_permille;     /* 0..1000 */
    uint16_t fps_x10;            /* 598 means 59.8 fps */
    uint8_t mode;
} BallLinkExtendedData;

typedef struct
{
    uint8_t buffer[BALL_LINK_LEGACY_FRAME_SIZE];
    uint8_t index;
} BallLinkLegacyParser;

typedef struct
{
    uint8_t buffer[BALL_LINK_EXTENDED_FRAME_SIZE];
    uint8_t index;
} BallLinkExtendedParser;

void BallLink_LegacyInit(BallLinkLegacyParser *parser);
bool BallLink_LegacyPushByte(BallLinkLegacyParser *parser,
                             uint8_t byte,
                             BallLinkLegacyData *out_data);

void BallLink_ExtendedInit(BallLinkExtendedParser *parser);
bool BallLink_ExtendedPushByte(BallLinkExtendedParser *parser,
                               uint8_t byte,
                               BallLinkExtendedData *out_data);

/* Compatibility aliases for code written for the old 12-byte parser. */
typedef BallLinkLegacyData BallLinkData;
typedef BallLinkLegacyParser BallLinkParser;
#define BALL_LINK_FRAME_SIZE BALL_LINK_LEGACY_FRAME_SIZE
void BallLink_Init(BallLinkParser *parser);
bool BallLink_PushByte(BallLinkParser *parser,
                       uint8_t byte,
                       BallLinkData *out_data);

size_t BallLink_BuildModeCommand(uint8_t mode, char *out, size_t out_size);
size_t BallLink_BuildTargetNormalizedCommand(uint16_t normalized_position,
                                             char *out,
                                             size_t out_size);
size_t BallLink_BuildTargetPixelCommand(uint16_t pixel_position,
                                        char *out,
                                        size_t out_size);
size_t BallLink_BuildCaptureCommand(char *out, size_t out_size);
size_t BallLink_BuildCenterCommand(char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* BALL_LINK_H_ */
