#ifndef REQ4_VISUAL_CONTROL_H_
#define REQ4_VISUAL_CONTROL_H_

#include <stdint.h>

typedef struct {
    /* Visual position controller. Units match BallBalanceConfig_t. */
    float kp_deg_per_cm;
    float ki_deg_per_cm_s;
    float kd_deg_per_cm_s;

    /* Predict the ball center forward by velocity * predict_time_s. */
    float predict_time_s;
    float predict_limit_cm;

    /* Requirement-4-only camera velocity filter. */
    float velocity_filter_alpha;

    /* Visual command limits; car acceleration feed-forward is added later. */
    float max_visual_angle_deg;
    float integral_limit_deg;
    float output_slew_deg_per_s;

    /* Camera-noise and screw static-friction handling. */
    float center_deadband_cm;

    /*
     * When the ball is nearly stationary inside this target band, static-
     * friction breakaway pulses are inhibited. Normal P/D control remains
     * active outside center_deadband_cm.
     */
    float breakaway_inhibit_band_cm;
    float breakaway_error_cm;
    float breakaway_speed_cm_s;
    float breakaway_angle_deg;
} Req4VisualConfig_t;

typedef struct {
    float target_ball_cm;
    float current_ball_cm;
    float error_cm;
    float raw_velocity_cm_s;
    float filtered_velocity_cm_s;
    float predicted_error_cm;

    float p_output_deg;
    float i_output_deg;
    float d_output_deg;
    float breakaway_output_deg;
    float raw_output_deg;
    float limited_output_deg;
    float output_flap_deg;

    uint8_t enabled;
    uint8_t breakaway_active;
    uint8_t breakaway_inhibited;
    uint8_t saturated;
    uint8_t in_one_cm_band;
} Req4VisualSnapshot_t;

void Req4VisualControl_Init(void);
void Req4VisualControl_Reset(void);
void Req4VisualControl_SetEnabled(uint8_t enable);
uint8_t Req4VisualControl_IsEnabled(void);

void Req4VisualControl_Update(
    float target_ball_cm,
    float current_ball_cm,
    float raw_velocity_cm_s,
    uint32_t delta_ms,
    int8_t control_sign);

void Req4VisualControl_GetConfig(Req4VisualConfig_t *config);
uint8_t Req4VisualControl_SetConfig(const Req4VisualConfig_t *config);
void Req4VisualControl_RestoreDefaults(void);
void Req4VisualControl_GetSnapshot(Req4VisualSnapshot_t *snapshot);

#endif /* REQ4_VISUAL_CONTROL_H_ */
