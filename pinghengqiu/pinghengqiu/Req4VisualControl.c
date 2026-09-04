#include "Req4VisualControl.h"

#include <string.h>

#define REQ4_BREAKAWAY_CONFIRM_MS      100U
#define REQ4_BREAKAWAY_PULSE_MS        180U
#define REQ4_BREAKAWAY_COOLDOWN_MS     260U
#define REQ4_IN_BAND_LIMIT_CM          1.0f

typedef struct {
    Req4VisualConfig_t config;
    Req4VisualSnapshot_t snapshot;

    float filtered_velocity_cm_s;
    float integral_error_cm_s;
    float previous_output_deg;

    uint32_t still_ms;
    uint32_t pulse_ms;
    uint32_t cooldown_ms;
    uint8_t enabled;
    uint8_t breakaway_active;
} Req4VisualContext_t;

static Req4VisualContext_t s_req4;

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float LimitFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float SignFloat(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}

static uint8_t IsFiniteFloat(float value)
{
    return (value == value && value > -1000000.0f && value < 1000000.0f) ? 1U : 0U;
}

static uint8_t ConfigIsValid(const Req4VisualConfig_t *config)
{
    if (config == 0) {
        return 0U;
    }

    if (!IsFiniteFloat(config->kp_deg_per_cm) ||
        !IsFiniteFloat(config->ki_deg_per_cm_s) ||
        !IsFiniteFloat(config->kd_deg_per_cm_s) ||
        !IsFiniteFloat(config->predict_time_s) ||
        !IsFiniteFloat(config->predict_limit_cm) ||
        !IsFiniteFloat(config->velocity_filter_alpha) ||
        !IsFiniteFloat(config->max_visual_angle_deg) ||
        !IsFiniteFloat(config->integral_limit_deg) ||
        !IsFiniteFloat(config->output_slew_deg_per_s) ||
        !IsFiniteFloat(config->center_deadband_cm) ||
        !IsFiniteFloat(config->breakaway_inhibit_band_cm) ||
        !IsFiniteFloat(config->breakaway_error_cm) ||
        !IsFiniteFloat(config->breakaway_speed_cm_s) ||
        !IsFiniteFloat(config->breakaway_angle_deg)) {
        return 0U;
    }

    if (config->kp_deg_per_cm < 0.0f || config->kp_deg_per_cm > 2.0f ||
        config->ki_deg_per_cm_s < 0.0f || config->ki_deg_per_cm_s > 1.0f ||
        config->kd_deg_per_cm_s < 0.0f || config->kd_deg_per_cm_s > 1.0f ||
        config->predict_time_s < 0.0f || config->predict_time_s > 0.50f ||
        config->predict_limit_cm < 0.0f || config->predict_limit_cm > 3.0f ||
        config->velocity_filter_alpha <= 0.0f || config->velocity_filter_alpha > 1.0f ||
        config->max_visual_angle_deg <= 0.0f || config->max_visual_angle_deg > 1.50f ||
        config->integral_limit_deg < 0.0f || config->integral_limit_deg > 1.0f ||
        config->output_slew_deg_per_s <= 0.0f || config->output_slew_deg_per_s > 50.0f ||
        config->center_deadband_cm < 0.0f || config->center_deadband_cm > 1.0f ||
        config->breakaway_inhibit_band_cm < 0.0f || config->breakaway_inhibit_band_cm > 2.0f ||
        config->breakaway_error_cm < 0.0f || config->breakaway_error_cm > 3.0f ||
        config->breakaway_speed_cm_s < 0.0f || config->breakaway_speed_cm_s > 5.0f ||
        config->breakaway_angle_deg < 0.0f || config->breakaway_angle_deg > 2.0f) {
        return 0U;
    }

    if (config->breakaway_angle_deg > config->max_visual_angle_deg ||
        config->breakaway_inhibit_band_cm < config->center_deadband_cm) {
        return 0U;
    }

    return 1U;
}

void Req4VisualControl_RestoreDefaults(void)
{
    /* Start from the successful requirement-3 visual gains, but keep all
     * requirement-4 values in a separate configuration block. */
    s_req4.config.kp_deg_per_cm = 0.20f;
    s_req4.config.ki_deg_per_cm_s = 0.0f;
    s_req4.config.kd_deg_per_cm_s = 0.06f;
    s_req4.config.predict_time_s = 0.08f;
    s_req4.config.predict_limit_cm = 0.80f;
    s_req4.config.velocity_filter_alpha = 0.22f;
    s_req4.config.max_visual_angle_deg = 0.90f;
    s_req4.config.integral_limit_deg = 0.10f;
    s_req4.config.output_slew_deg_per_s = 8.0f;
    s_req4.config.center_deadband_cm = 0.08f;
    s_req4.config.breakaway_inhibit_band_cm = 0.75f;
    s_req4.config.breakaway_error_cm = 0.35f;
    s_req4.config.breakaway_speed_cm_s = 0.25f;
    s_req4.config.breakaway_angle_deg = 0.55f;
}

void Req4VisualControl_Init(void)
{
    memset(&s_req4, 0, sizeof(s_req4));
    Req4VisualControl_RestoreDefaults();
}

void Req4VisualControl_Reset(void)
{
    s_req4.filtered_velocity_cm_s = 0.0f;
    s_req4.integral_error_cm_s = 0.0f;
    s_req4.previous_output_deg = 0.0f;
    s_req4.still_ms = 0U;
    s_req4.pulse_ms = 0U;
    s_req4.cooldown_ms = 0U;
    s_req4.breakaway_active = 0U;
    memset(&s_req4.snapshot, 0, sizeof(s_req4.snapshot));
    s_req4.snapshot.enabled = s_req4.enabled;
}

void Req4VisualControl_SetEnabled(uint8_t enable)
{
    s_req4.enabled = (enable != 0U) ? 1U : 0U;
    Req4VisualControl_Reset();
}

uint8_t Req4VisualControl_IsEnabled(void)
{
    return s_req4.enabled;
}

void Req4VisualControl_Update(
    float target_ball_cm,
    float current_ball_cm,
    float raw_velocity_cm_s,
    uint32_t delta_ms,
    int8_t control_sign)
{
    const Req4VisualConfig_t *config = &s_req4.config;
    float dt_s;
    float error_cm;
    float prediction_cm;
    float predicted_error_cm;
    float p_output;
    float i_output;
    float d_output;
    float breakaway_output = 0.0f;
    float integral_candidate;
    float raw_output;
    float limited_output;
    float signed_output;
    float max_step;
    float output;
    uint8_t saturated;
    uint8_t breakaway_inhibited;

    if (s_req4.enabled == 0U ||
        !IsFiniteFloat(target_ball_cm) ||
        !IsFiniteFloat(current_ball_cm) ||
        !IsFiniteFloat(raw_velocity_cm_s)) {
        Req4VisualControl_Reset();
        return;
    }

    if (delta_ms < 5U) {
        delta_ms = 5U;
    } else if (delta_ms > 200U) {
        delta_ms = 200U;
    }
    dt_s = (float)delta_ms * 0.001f;

    s_req4.filtered_velocity_cm_s +=
        config->velocity_filter_alpha *
        (raw_velocity_cm_s - s_req4.filtered_velocity_cm_s);

    error_cm = target_ball_cm - current_ball_cm;
    prediction_cm = s_req4.filtered_velocity_cm_s * config->predict_time_s;
    prediction_cm = LimitFloat(
        prediction_cm,
        -config->predict_limit_cm,
        config->predict_limit_cm);
    predicted_error_cm = target_ball_cm - (current_ball_cm + prediction_cm);

    p_output = config->kp_deg_per_cm * predicted_error_cm;
    d_output = -config->kd_deg_per_cm_s * s_req4.filtered_velocity_cm_s;

    integral_candidate = s_req4.integral_error_cm_s + error_cm * dt_s;
    if (config->ki_deg_per_cm_s > 0.0f) {
        float max_integral = config->integral_limit_deg / config->ki_deg_per_cm_s;
        integral_candidate = LimitFloat(
            integral_candidate,
            -max_integral,
            max_integral);
        i_output = config->ki_deg_per_cm_s * integral_candidate;
    } else {
        integral_candidate = 0.0f;
        i_output = 0.0f;
    }

    if (s_req4.cooldown_ms > delta_ms) {
        s_req4.cooldown_ms -= delta_ms;
    } else {
        s_req4.cooldown_ms = 0U;
    }

    /*
     * Do not kick a ball that is already acceptably close to the target and
     * nearly stationary. This only suppresses the static-friction pulse;
     * normal visual P/D control remains available for small corrections.
     */
    breakaway_inhibited =
        (AbsFloat(error_cm) <= config->breakaway_inhibit_band_cm) &&
        (AbsFloat(s_req4.filtered_velocity_cm_s) <=
         config->breakaway_speed_cm_s);

    if (breakaway_inhibited != 0U) {
        s_req4.still_ms = 0U;
        s_req4.breakaway_active = 0U;
        s_req4.pulse_ms = 0U;
    } else if (AbsFloat(error_cm) >= config->breakaway_error_cm &&
               AbsFloat(s_req4.filtered_velocity_cm_s) <=
                   config->breakaway_speed_cm_s) {
        if (s_req4.still_ms < REQ4_BREAKAWAY_CONFIRM_MS) {
            s_req4.still_ms += delta_ms;
        }
    } else {
        s_req4.still_ms = 0U;
        if (s_req4.breakaway_active != 0U &&
            AbsFloat(s_req4.filtered_velocity_cm_s) >
                config->breakaway_speed_cm_s) {
            s_req4.breakaway_active = 0U;
            s_req4.pulse_ms = 0U;
            s_req4.cooldown_ms = REQ4_BREAKAWAY_COOLDOWN_MS;
        }
    }

    if (s_req4.breakaway_active == 0U &&
        s_req4.cooldown_ms == 0U &&
        s_req4.still_ms >= REQ4_BREAKAWAY_CONFIRM_MS) {
        s_req4.breakaway_active = 1U;
        s_req4.pulse_ms = 0U;
    }

    if (s_req4.breakaway_active != 0U) {
        s_req4.pulse_ms += delta_ms;
        breakaway_output = SignFloat(error_cm) * config->breakaway_angle_deg;
        if (s_req4.pulse_ms >= REQ4_BREAKAWAY_PULSE_MS) {
            s_req4.breakaway_active = 0U;
            s_req4.pulse_ms = 0U;
            s_req4.still_ms = 0U;
            s_req4.cooldown_ms = REQ4_BREAKAWAY_COOLDOWN_MS;
        }
    }

    raw_output = p_output + i_output + d_output;
    if (breakaway_output != 0.0f &&
        SignFloat(raw_output) == SignFloat(breakaway_output) &&
        AbsFloat(raw_output) < AbsFloat(breakaway_output)) {
        raw_output = breakaway_output;
    }

    if (AbsFloat(error_cm) <= config->center_deadband_cm &&
        AbsFloat(s_req4.filtered_velocity_cm_s) <= config->breakaway_speed_cm_s) {
        raw_output = 0.0f;
        integral_candidate *= 0.80f;
        i_output = config->ki_deg_per_cm_s * integral_candidate;
    }

    limited_output = LimitFloat(
        raw_output,
        -config->max_visual_angle_deg,
        config->max_visual_angle_deg);
    saturated = (limited_output != raw_output) ? 1U : 0U;

    /* Only integrate when not pushing farther into saturation. */
    if (saturated == 0U ||
        SignFloat(error_cm) != SignFloat(raw_output)) {
        s_req4.integral_error_cm_s = integral_candidate;
    }

    signed_output = (float)((control_sign < 0) ? -1 : 1) * limited_output;
    max_step = config->output_slew_deg_per_s * dt_s;
    output = LimitFloat(
        signed_output,
        s_req4.previous_output_deg - max_step,
        s_req4.previous_output_deg + max_step);
    s_req4.previous_output_deg = output;

    s_req4.snapshot.target_ball_cm = target_ball_cm;
    s_req4.snapshot.current_ball_cm = current_ball_cm;
    s_req4.snapshot.error_cm = error_cm;
    s_req4.snapshot.raw_velocity_cm_s = raw_velocity_cm_s;
    s_req4.snapshot.filtered_velocity_cm_s = s_req4.filtered_velocity_cm_s;
    s_req4.snapshot.predicted_error_cm = predicted_error_cm;
    s_req4.snapshot.p_output_deg = p_output;
    s_req4.snapshot.i_output_deg = i_output;
    s_req4.snapshot.d_output_deg = d_output;
    s_req4.snapshot.breakaway_output_deg = breakaway_output;
    s_req4.snapshot.raw_output_deg = raw_output;
    s_req4.snapshot.limited_output_deg = limited_output;
    s_req4.snapshot.output_flap_deg = output;
    s_req4.snapshot.enabled = s_req4.enabled;
    s_req4.snapshot.breakaway_active = s_req4.breakaway_active;
    s_req4.snapshot.breakaway_inhibited = breakaway_inhibited;
    s_req4.snapshot.saturated = saturated;
    s_req4.snapshot.in_one_cm_band =
        (AbsFloat(error_cm) <= REQ4_IN_BAND_LIMIT_CM) ? 1U : 0U;
}

void Req4VisualControl_GetConfig(Req4VisualConfig_t *config)
{
    if (config != 0) {
        *config = s_req4.config;
    }
}

uint8_t Req4VisualControl_SetConfig(const Req4VisualConfig_t *config)
{
    if (ConfigIsValid(config) == 0U) {
        return 0U;
    }

    s_req4.config = *config;
    Req4VisualControl_Reset();
    return 1U;
}

void Req4VisualControl_GetSnapshot(Req4VisualSnapshot_t *snapshot)
{
    if (snapshot != 0) {
        *snapshot = s_req4.snapshot;
    }
}
