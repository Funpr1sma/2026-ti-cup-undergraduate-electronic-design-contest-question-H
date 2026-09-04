#ifndef BALL_LINK_H_
#define BALL_LINK_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BALL_LINK_FRAME_SIZE       12U
#define BALL_LINK_INVALID_POSITION 0xFFFFU
#define BALL_LINK_POSITION_CENTER  5000U
#define BALL_LINK_POSITION_MAX     10000U

#define BALL_LINK_FLAG_VALID       (1U << 0)
#define BALL_LINK_FLAG_TARGET_LOCK (1U << 1)
#define BALL_LINK_FLAG_RTSP_ON     (1U << 2)

typedef struct
{
    uint8_t sequence;
    uint8_t flags;
    uint16_t ball_position;
    uint16_t target_position;
    uint16_t score_permille;
    uint8_t mode;
} BallLinkData;

typedef struct
{
    uint8_t buffer[BALL_LINK_FRAME_SIZE];
    uint8_t index;
} BallLinkParser;

void BallLink_Init(BallLinkParser *parser);
bool BallLink_PushByte(BallLinkParser *parser,
                       uint8_t byte,
                       BallLinkData *out_data);

size_t BallLink_BuildModeCommand(uint8_t mode, char *out, size_t out_size);
size_t BallLink_BuildTargetNormalizedCommand(uint16_t normalized_position,
                                             char *out,
                                             size_t out_size);
size_t BallLink_BuildCaptureCommand(char *out, size_t out_size);
size_t BallLink_BuildCenterCommand(char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* BALL_LINK_H_ */
