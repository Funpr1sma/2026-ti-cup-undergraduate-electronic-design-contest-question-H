#include "ball_balance_control.h"

#include <math.h>
#include <string.h>

#include "clock.h"
#include "closed_loop.h"
#include "motor.h"
#include "Req4VisualControl.h"
#include "ti_msp_dl_config.h"

#define BALL_BALANCE_PERIOD_S       0.020f
#define BALL_BALANCE_PERIOD_MS      20U
#define BALL_BALANCE_RAD_TO_DEG     57.2957795131f
#define BALL_BALANCE_DEG_TO_RAD     0.01745329252f
#define BALL_BALANCE_CAMERA_TIMEOUT_DEFAULT_MS 1000U

/*
 * Stateful static-friction compensation.
 *
 * max_flap_angle_deg remains the normal-motion angle limit. When the ball
 * is confirmed stationary, the controller uses either a short 0.74 degree
 * near-target pulse or the direction-dependent far breakaway pulse. As soon
 * as the measured
 * displacement proves that the ball has started moving, normal PID resumes.
 */
#define BALL_BALANCE_BREAKAWAY_TRIGGER_ERROR_CM  0.45f
#define BALL_BALANCE_BREAKAWAY_SOFT_MAX_ERROR_CM  1.00f
#define BALL_BALANCE_BREAKAWAY_SOFT_ANGLE_DEG      0.74f
#define BALL_BALANCE_BREAKAWAY_SOFT_MAX_MS         280U
#define BALL_BALANCE_BREAKAWAY_STILL_SPEED_CM_S  0.22f
#define BALL_BALANCE_BREAKAWAY_CONFIRM_MS         120U
#define BALL_BALANCE_BREAKAWAY_POS_FAR_DEG         1.18f
#define BALL_BALANCE_BREAKAWAY_NEG_FAR_DEG         1.08f
#define BALL_BALANCE_BREAKAWAY_POS_NEAR_DEG        0.96f
#define BALL_BALANCE_BREAKAWAY_NEG_NEAR_DEG        0.88f
#define BALL_BALANCE_BREAKAWAY_FULL_ERROR_CM       2.20f
#define BALL_BALANCE_BREAKAWAY_EXIT_MOVE_CM        0.08f
#define BALL_BALANCE_BREAKAWAY_FAR_MAX_MS          500U
#define BALL_BALANCE_BREAKAWAY_COOLDOWN_MS         220U

/*
 * Direction-dependent drive limits measured from the levelled mechanism.
 * Error > 0 drives the ball toward +5 cm; error < 0 drives toward -5 cm.
 * Braking is not restricted by these values and may use maxangle in full.
 */
#define BALL_BALANCE_POSITIVE_DRIVE_LIMIT_DEG      1.18f
#define BALL_BALANCE_NEGATIVE_DRIVE_LIMIT_DEG      1.08f

/*
 * Direction-dependent prediction.
 *
 * The +5 cm leg in requirement 3 is a pass-through move: the measured BALL
 * CENTER must enter the +5 cm band and the target is then changed immediately
 * to -5 cm.  A long prediction on that leg made the platform reverse while the
 * center was only near +2.2 cm.  Disable prediction toward +5 cm and use the
 * dedicated ball-center transit schedule below; retain 130 ms toward -5 cm
 * where the ball must finally settle.
 */
#define BALL_BALANCE_POS_BRAKE_LEAD_TIME_S          0.00f
#define BALL_BALANCE_POS_BRAKE_LEAD_MAX_CM          0.00f
#define BALL_BALANCE_NEG_BRAKE_LEAD_TIME_S          0.13f
#define BALL_BALANCE_NEG_BRAKE_LEAD_MAX_CM          1.10f

/* Direction-dependent distance-to-speed profiles. */
#define BALL_BALANCE_POS_SPEED_PROFILE_GAIN         1.35f
#define BALL_BALANCE_POS_SPEED_PROFILE_MAX_CM_S     4.20f
#define BALL_BALANCE_POS_SPEED_TRACK_GAIN           0.18f
#define BALL_BALANCE_NEG_SPEED_PROFILE_GAIN         1.05f
#define BALL_BALANCE_NEG_SPEED_PROFILE_MAX_CM_S     3.50f
#define BALL_BALANCE_NEG_SPEED_TRACK_GAIN           0.27f
#define BALL_BALANCE_VELOCITY_TERM_LIMIT_DEG         1.35f

/*
 * Requirement-3 +5 cm leg is a pass-through motion based on BALL CENTER.
 * The old 0.24 degree floor still let the screw platform return almost to
 * level from ball_center ~= 0.9 cm, so the ball stalled around +3 cm.
 *
 * Hold the full +5-direction tilt until the center reaches +3.55 cm, then
 * taper only the forward drive to 0.38 degree at +4.45 cm.  No reverse brake
 * is allowed before the center enters the +5 cm switching band.
 */
#define BALL_BALANCE_POS_TRANSIT_FULL_END_CM         3.55f
#define BALL_BALANCE_POS_TRANSIT_TAPER_END_CM        4.45f
#define BALL_BALANCE_POS_TRANSIT_END_DRIVE_DEG       0.38f
#define BALL_BALANCE_POS_TRANSIT_SWITCH_MIN_CM       4.50f

/*
 * +5 cm actuator pre-positioning.
 *
 * Do not command reverse tilt before the measured BALL CENTER enters the
 * legal switching band (4.50..5.50 cm).  Instead, when the forward prediction says the ball
 * will soon reach +5 cm, command mechanical level so the slow screw has less
 * distance to travel after the +5 -> -5 target change.  If the ball becomes
 * too slow before entering the band, use only a small forward creep command.
 */
#define BALL_BALANCE_POS_PREPOSITION_PREDICT_TIME_S   1.15f
#define BALL_BALANCE_POS_PREPOSITION_TRIGGER_CM       4.55f
#define BALL_BALANCE_POS_PREPOSITION_MIN_CENTER_CM    1.20f
#define BALL_BALANCE_POS_PREPOSITION_MIN_SPEED_CM_S   2.00f
#define BALL_BALANCE_POS_PREPOSITION_CREEP_SPEED_CM_S 0.90f
#define BALL_BALANCE_POS_PREPOSITION_CREEP_DRIVE_DEG  0.22f
#define BALL_BALANCE_POS_PREPOSITION_BRAKE_START_CM    3.70f
#define BALL_BALANCE_POS_PREPOSITION_BRAKE_DEG         0.12f

/*
 * The 235 mm arm and 2 mm-lead screw need noticeable time to reverse.  When
 * the stopping-distance estimate says the ball can no longer stop inside the
 * remaining distance, request full opposite tilt immediately.  The +5 cm
 * pass-through guard above has priority until the center reaches its band.
 */
#define BALL_BALANCE_STOP_DECEL_CM_S2                6.00f
#define BALL_BALANCE_STOP_ACTUATOR_DELAY_S           0.20f
#define BALL_BALANCE_STOP_MARGIN_CM                   0.15f
#define BALL_BALANCE_STOP_MIN_SPEED_CM_S              3.80f

/*
 * Requirement-3 +5 -> -5 velocity-phase controller.
 *
 * 1. Reverse the ball with full tilt.
 * 2. Once velocity has changed toward -5 cm, ramp the drive tilt toward
 *    the 0.28 degree cruise bias.
 * 3. Use a smooth speed limiter instead of toggling between drive and brake.
 * 4. Predict the BALL CENTER 0.24 s ahead.  The previous 0.54 s horizon
 *    started braking near -0.8 cm and stopped the ball near -2.6 cm.
 * 5. Near the target, taper and release the milder two-stage brake early.
 */
#define BALL_BALANCE_R3_VELOCITY_FLIP_MIN_CM_S        0.60f
#define BALL_BALANCE_R3_INITIAL_REVERSE_ANGLE_DEG     1.35f
#define BALL_BALANCE_R3_DRIVE_RAMP_MS                 180U
#define BALL_BALANCE_R3_COAST_DRIVE_DEG               0.33f
#define BALL_BALANCE_R3_SPEED_CAP_START_CM_S           6.40f
#define BALL_BALANCE_R3_SPEED_CAP_FULL_CM_S            7.40f
#define BALL_BALANCE_R3_SPEED_CAP_BRAKE_DEG            0.16f
#define BALL_BALANCE_R3_PREDICT_TIME_S                 0.24f
#define BALL_BALANCE_R3_PREDICT_BAND_CM                0.45f

/*
 * If the ball slows to a stop while it is still more than 1.2 cm from -5 cm,
 * briefly raise the cruise bias.  This is intentionally local to the
 * requirement-3 reverse-drive phase, because the generic breakaway controller
 * is disabled while that phase owns the actuator command.
 */
#define BALL_BALANCE_R3_STALL_SPEED_CM_S                0.38f
#define BALL_BALANCE_R3_STALL_EXIT_SPEED_CM_S           0.90f
#define BALL_BALANCE_R3_STALL_ERROR_CM                  1.20f
#define BALL_BALANCE_R3_STALL_CONFIRM_MS                140U
#define BALL_BALANCE_R3_STALL_BOOST_DEG                 0.42f

/*
 * Two-stage actuator-aware braking.
 *
 * The screw must initially traverse quickly from the drive side, so retain a
 * short full-angle attack.  Once the measured mechanism has actually built
 * about 0.32 degree of brake, reduce the target to 0.55 degree.  This keeps
 * fast actuator traversal without parking the mechanism at a large brake
 * angle until the ball velocity reverses.
 */
#define BALL_BALANCE_R3_BRAKE_ATTACK_ANGLE_DEG        0.80f
#define BALL_BALANCE_R3_BRAKE_HOLD_ANGLE_DEG          0.55f
#define BALL_BALANCE_R3_BRAKE_ACTUAL_SWITCH_DEG       0.32f
#define BALL_BALANCE_R3_BRAKE_REVERSE_MIN_CM_S        0.04f
#define BALL_BALANCE_R3_BRAKE_RELEASE_START_CM_S      1.60f
#define BALL_BALANCE_R3_BRAKE_RELEASE_ERROR_CM        1.10f
#define BALL_BALANCE_R3_BRAKE_RELEASE_MS               50U
#define BALL_BALANCE_R3_BRAKE_STOP_SPEED_CM_S         0.30f
#define BALL_BALANCE_R3_BRAKE_TAPER_SPEED_CM_S        3.40f
#define BALL_BALANCE_R3_BRAKE_TAPER_MIN_RATIO         0.05f
#define BALL_BALANCE_R3_BRAKE_TAPER_ERROR_CM          1.50f
#define BALL_BALANCE_R3_FINE_ENTRY_ERROR_CM           0.60f

/*
 * After the brake is released, command mechanical level briefly before the
 * normal fine controller is allowed to pull in the opposite direction.
 * The phase may finish early once the measured flap is close to level.
 */
#define BALL_BALANCE_R3_UNLOAD_MIN_MS                  50U
#define BALL_BALANCE_R3_UNLOAD_MAX_MS                 260U
#define BALL_BALANCE_R3_UNLOAD_MOTOR_WINDOW_DEG       0.16f

#define BALL_BALANCE_R3_SETTLE_MAX_ANGLE_DEG          0.34f
#define BALL_BALANCE_R3_FINE_BREAKAWAY_MAX_DEG        0.52f

/* Taper only the drive command near the target; never taper braking. */
#define BALL_BALANCE_APPROACH_START_ERROR_CM       1.35f
#define BALL_BALANCE_APPROACH_NEAR_ERROR_CM        0.55f
#define BALL_BALANCE_APPROACH_NEAR_LIMIT_DEG       0.32f

#define BALL_BALANCE_TARGET_DEADBAND_CM            0.12f

typedef enum {
    BALL_BALANCE_R3_PHASE_NONE = 0,
    BALL_BALANCE_R3_PHASE_REVERSE_DRIVE,
    BALL_BALANCE_R3_PHASE_PREDICTIVE_BRAKE,
    BALL_BALANCE_R3_PHASE_BRAKE_UNLOAD,
    BALL_BALANCE_R3_PHASE_FINE_SETTLE
} BallBalanceR3Phase_t;

typedef struct {
    BallBalanceConfig_t config;

    volatile float target_ball_cm;
    volatile uint8_t enabled;
    volatile uint8_t level_calibrated;

    volatile float external_feedforward_angle_deg;
    volatile float external_feedforward_acceleration_mps2;
    volatile uint8_t external_feedforward_enabled;
    volatile uint8_t external_feedforward_valid;
    volatile uint8_t requirement4_mode;

    volatile float measured_ball_cm;
    volatile uint32_t measurement_timestamp_ms;
    volatile uint32_t measurement_sequence;

    uint32_t processed_sequence;
    uint32_t previous_measurement_ms;

    float previous_ball_cm;
    float filtered_velocity_cm_s;
    float integral_error_cm_s;

    uint32_t breakaway_still_ms;
    uint32_t breakaway_elapsed_ms;
    uint32_t breakaway_cooldown_ms;
    float breakaway_start_ball_cm;
    uint8_t breakaway_active;
    uint8_t breakaway_soft;

    uint32_t settle_start_ms;
    uint8_t settle_timer_active;
    volatile uint8_t settled;

    BallBalanceR3Phase_t r3_phase;
    uint32_t r3_toward_velocity_ms;
    uint32_t r3_brake_reverse_ms;
    uint32_t r3_unload_ms;
    uint32_t r3_stall_ms;
    uint8_t positive_preposition_active;
    int8_t r3_target_direction_sign;
    int8_t r3_brake_controller_sign;

    volatile BallBalanceState_t state;
    volatile BallBalanceFault_t fault;

    BallBalanceSnapshot_t snapshot;
} BallBalanceContext_t;

static BallBalanceContext_t s_balance;

static float BallBalance_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static float BallBalance_Limit(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static uint8_t BallBalance_IsFinite(float value)
{
    return value == value &&
           value > -1000000.0f &&
           value < 1000000.0f;
}

static float BallBalance_Sign(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    }

    if (value < 0.0f) {
        return -1.0f;
    }

    return 0.0f;
}

/*
 * Reduce the normal-motion angle limit as the ball approaches the target.
 * This is deliberately based on remaining distance rather than time, so the
 * same file works for both +5 cm and -5 cm moves.
 */
static float BallBalance_GetApproachAngleLimit(
    float configured_limit_deg,
    float abs_error_cm)
{
    float near_limit_deg;
    float ratio;

    near_limit_deg = BALL_BALANCE_APPROACH_NEAR_LIMIT_DEG;
    if (near_limit_deg > configured_limit_deg) {
        near_limit_deg = configured_limit_deg;
    }

    if (abs_error_cm >= BALL_BALANCE_APPROACH_START_ERROR_CM) {
        return configured_limit_deg;
    }

    if (abs_error_cm <= BALL_BALANCE_APPROACH_NEAR_ERROR_CM) {
        return near_limit_deg;
    }

    ratio =
        (abs_error_cm - BALL_BALANCE_APPROACH_NEAR_ERROR_CM) /
        (BALL_BALANCE_APPROACH_START_ERROR_CM -
         BALL_BALANCE_APPROACH_NEAR_ERROR_CM);

    return near_limit_deg +
           ratio * (configured_limit_deg - near_limit_deg);
}

/*
 * Minimum forward controller output for the +5 cm pass-through leg.
 * Inputs and thresholds are camera BALL CENTER coordinates in centimetres.
 */
static float BallBalance_GetPositiveTransitMinDrive(
    float ball_center_cm,
    float configured_drive_limit_deg)
{
    float ratio;
    float minimum_drive_deg;

    if (ball_center_cm <= BALL_BALANCE_POS_TRANSIT_FULL_END_CM) {
        return configured_drive_limit_deg;
    }

    if (ball_center_cm >= BALL_BALANCE_POS_TRANSIT_TAPER_END_CM) {
        return BALL_BALANCE_POS_TRANSIT_END_DRIVE_DEG;
    }

    ratio =
        (BALL_BALANCE_POS_TRANSIT_TAPER_END_CM - ball_center_cm) /
        (BALL_BALANCE_POS_TRANSIT_TAPER_END_CM -
         BALL_BALANCE_POS_TRANSIT_FULL_END_CM);

    minimum_drive_deg =
        BALL_BALANCE_POS_TRANSIT_END_DRIVE_DEG +
        ratio * (configured_drive_limit_deg -
                 BALL_BALANCE_POS_TRANSIT_END_DRIVE_DEG);

    return BallBalance_Limit(
        minimum_drive_deg,
        BALL_BALANCE_POS_TRANSIT_END_DRIVE_DEG,
        configured_drive_limit_deg);
}

/*
 * Use a fixed soft pulse just outside the final settling band and up to 1.00 cm error. Farther away, use
 * the direction-dependent static-friction angle. This avoids using integral
 * action to overcome stiction near the target.
 */
static float BallBalance_GetBreakawayAngle(float error_cm, uint8_t soft_mode)
{
    float abs_error_cm;
    float far_angle_deg;
    float near_angle_deg;
    float ratio;

    abs_error_cm = BallBalance_Abs(error_cm);

    if (soft_mode != 0U) {
        return BALL_BALANCE_BREAKAWAY_SOFT_ANGLE_DEG;
    }

    if (error_cm >= 0.0f) {
        far_angle_deg = BALL_BALANCE_BREAKAWAY_POS_FAR_DEG;
        near_angle_deg = BALL_BALANCE_BREAKAWAY_POS_NEAR_DEG;
    } else {
        far_angle_deg = BALL_BALANCE_BREAKAWAY_NEG_FAR_DEG;
        near_angle_deg = BALL_BALANCE_BREAKAWAY_NEG_NEAR_DEG;
    }

    if (abs_error_cm >= BALL_BALANCE_BREAKAWAY_FULL_ERROR_CM) {
        return far_angle_deg;
    }

    if (abs_error_cm <= BALL_BALANCE_BREAKAWAY_TRIGGER_ERROR_CM) {
        return near_angle_deg;
    }

    ratio =
        (abs_error_cm - BALL_BALANCE_BREAKAWAY_TRIGGER_ERROR_CM) /
        (BALL_BALANCE_BREAKAWAY_FULL_ERROR_CM -
         BALL_BALANCE_BREAKAWAY_TRIGGER_ERROR_CM);

    return near_angle_deg +
           ratio * (far_angle_deg - near_angle_deg);
}
static void BallBalance_ReadMeasurement(
    float *position_cm,
    uint32_t *timestamp_ms,
    uint32_t *sequence)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (position_cm != 0) {
        *position_cm =
            s_balance.measured_ball_cm;
    }

    if (timestamp_ms != 0) {
        *timestamp_ms =
            s_balance.measurement_timestamp_ms;
    }

    if (sequence != 0) {
        *sequence =
            s_balance.measurement_sequence;
    }

    if (primask == 0U) {
        __enable_irq();
    }
}

static void BallBalance_ResetRequirement3Motion(void)
{
    s_balance.r3_phase = BALL_BALANCE_R3_PHASE_NONE;
    s_balance.r3_toward_velocity_ms = 0U;
    s_balance.r3_brake_reverse_ms = 0U;
    s_balance.r3_unload_ms = 0U;
    s_balance.r3_stall_ms = 0U;
    s_balance.positive_preposition_active = 0U;
    s_balance.r3_target_direction_sign = 0;
    s_balance.r3_brake_controller_sign = 0;
}

/*
 * Special motion shaping for the requirement-3 +5 cm -> -5 cm reversal.
 *
 * Return 1 when controller_output has been overridden by this state machine.
 * The target and all position tests are BALL CENTER coordinates.
 */
static uint8_t BallBalance_UpdateRequirement3Motion(
    float target_cm,
    float ball_cm,
    float error_cm,
    float velocity_cm_s,
    uint32_t elapsed_ms,
    float configured_max_angle_deg,
    float motor_current_flap_deg,
    float *controller_output,
    uint8_t *braking_command)
{
    float moving_toward;
    float projected_center_cm;
    float projected_error_cm;
    float ramp_ratio;
    float drive_angle_deg;
    float release_ratio;
    float brake_taper_ratio;
    float brake_angle_deg;
    float desired_physical_brake_sign;
    float actual_brake_progress_deg;
    float toward_speed_cm_s;
    float speed_cap_ratio;
    uint8_t release_brake;

    if (controller_output == 0 || braking_command == 0) {
        return 0U;
    }

    if (s_balance.r3_phase == BALL_BALANCE_R3_PHASE_NONE) {
        return 0U;
    }

    if (target_cm >= 0.0f || error_cm == 0.0f) {
        BallBalance_ResetRequirement3Motion();
        return 0U;
    }

    if (elapsed_ms > BALL_BALANCE_CAMERA_TIMEOUT_DEFAULT_MS) {
        elapsed_ms = BALL_BALANCE_PERIOD_MS;
    }

    if (s_balance.r3_target_direction_sign == 0) {
        s_balance.r3_target_direction_sign =
            (error_cm >= 0.0f) ? 1 : -1;
    }

    moving_toward =
        ((float)s_balance.r3_target_direction_sign *
             velocity_cm_s > 0.0f) ? 1.0f : 0.0f;

    if (s_balance.r3_phase ==
            BALL_BALANCE_R3_PHASE_REVERSE_DRIVE) {
        /*
         * Cancel the old + direction velocity with a near-full 1.35 degree
         * command.  Once the ball is definitely moving toward -5 cm, ramp to
         * a 0.28 degree cruise bias instead of almost level.  The cruise bias
         * prevents the ball from stopping around -1...-2 cm, while the speed
         * cap below keeps the negative velocity from growing without bound.
         */
        if (moving_toward > 0.5f &&
            BallBalance_Abs(velocity_cm_s) >=
                BALL_BALANCE_R3_VELOCITY_FLIP_MIN_CM_S) {
            if (s_balance.r3_toward_velocity_ms <
                    BALL_BALANCE_R3_DRIVE_RAMP_MS) {
                s_balance.r3_toward_velocity_ms += elapsed_ms;
                if (s_balance.r3_toward_velocity_ms >
                        BALL_BALANCE_R3_DRIVE_RAMP_MS) {
                    s_balance.r3_toward_velocity_ms =
                        BALL_BALANCE_R3_DRIVE_RAMP_MS;
                }
            }
        } else if ((float)s_balance.r3_target_direction_sign *
                       velocity_cm_s <
                   -BALL_BALANCE_R3_VELOCITY_FLIP_MIN_CM_S) {
            s_balance.r3_toward_velocity_ms = 0U;
        }

        if (s_balance.r3_toward_velocity_ms == 0U) {
            drive_angle_deg =
                BALL_BALANCE_R3_INITIAL_REVERSE_ANGLE_DEG;
            if (drive_angle_deg > configured_max_angle_deg) {
                drive_angle_deg = configured_max_angle_deg;
            }
        } else {
            ramp_ratio =
                (float)s_balance.r3_toward_velocity_ms /
                (float)BALL_BALANCE_R3_DRIVE_RAMP_MS;
            ramp_ratio = BallBalance_Limit(ramp_ratio, 0.0f, 1.0f);

            drive_angle_deg =
                BALL_BALANCE_R3_COAST_DRIVE_DEG +
                (1.0f - ramp_ratio) *
                (BALL_BALANCE_NEGATIVE_DRIVE_LIMIT_DEG -
                 BALL_BALANCE_R3_COAST_DRIVE_DEG);
        }

        *controller_output =
            BallBalance_Sign(error_cm) * drive_angle_deg;

        /*
         * Until velocity has reversed, this command is braking the old motion
         * and may use the full configured maxangle.
         */
        *braking_command =
            (s_balance.r3_toward_velocity_ms == 0U) ? 1U : 0U;

        toward_speed_cm_s =
            (float)s_balance.r3_target_direction_sign * velocity_cm_s;

        /*
         * Smooth speed limiting.
         *
         * The previous hard 6.8 cm/s switch alternated the command between
         * +0.22 and -0.18 degree on consecutive camera frames.  Blend through
         * zero from 6.4 to 7.4 cm/s so the screw does not repeatedly reverse.
         */
        if (s_balance.r3_toward_velocity_ms != 0U &&
            toward_speed_cm_s >
                BALL_BALANCE_R3_SPEED_CAP_START_CM_S) {
            speed_cap_ratio =
                (toward_speed_cm_s -
                 BALL_BALANCE_R3_SPEED_CAP_START_CM_S) /
                (BALL_BALANCE_R3_SPEED_CAP_FULL_CM_S -
                 BALL_BALANCE_R3_SPEED_CAP_START_CM_S);
            speed_cap_ratio =
                BallBalance_Limit(speed_cap_ratio, 0.0f, 1.0f);

            *controller_output =
                BallBalance_Sign(error_cm) *
                    BALL_BALANCE_R3_COAST_DRIVE_DEG *
                    (1.0f - speed_cap_ratio) -
                BallBalance_Sign(error_cm) *
                    BALL_BALANCE_R3_SPEED_CAP_BRAKE_DEG *
                    speed_cap_ratio;

            *braking_command =
                (speed_cap_ratio >= 0.55f) ? 1U : 0U;
        }

        /*
         * The uploaded log reaches about -3 cm and then remains stationary
         * with a measured 0.22 degree cruise command.  Confirm the stall before
         * applying a modest 0.42 degree restart pulse.  As soon as the ball
         * speed recovers, return to the normal 0.28 degree cruise and speed cap.
         */
        if (s_balance.r3_toward_velocity_ms >=
                BALL_BALANCE_R3_DRIVE_RAMP_MS &&
            BallBalance_Abs(error_cm) >=
                BALL_BALANCE_R3_STALL_ERROR_CM) {
            if (toward_speed_cm_s <=
                    BALL_BALANCE_R3_STALL_SPEED_CM_S) {
                if (s_balance.r3_stall_ms <
                        BALL_BALANCE_R3_STALL_CONFIRM_MS) {
                    s_balance.r3_stall_ms += elapsed_ms;
                    if (s_balance.r3_stall_ms >
                            BALL_BALANCE_R3_STALL_CONFIRM_MS) {
                        s_balance.r3_stall_ms =
                            BALL_BALANCE_R3_STALL_CONFIRM_MS;
                    }
                }
            } else if (toward_speed_cm_s >=
                       BALL_BALANCE_R3_STALL_EXIT_SPEED_CM_S) {
                s_balance.r3_stall_ms = 0U;
            }
        } else {
            s_balance.r3_stall_ms = 0U;
        }

        if (s_balance.r3_stall_ms >=
                BALL_BALANCE_R3_STALL_CONFIRM_MS) {
            *controller_output =
                BallBalance_Sign(error_cm) *
                BALL_BALANCE_R3_STALL_BOOST_DEG;
            *braking_command = 0U;
        }

        projected_center_cm =
            ball_cm +
            velocity_cm_s * BALL_BALANCE_R3_PREDICT_TIME_S;
        projected_error_cm = target_cm - projected_center_cm;

        if (moving_toward > 0.5f &&
            (BallBalance_Abs(error_cm) <=
                 BALL_BALANCE_R3_PREDICT_BAND_CM ||
             BallBalance_Abs(projected_error_cm) <=
                 BALL_BALANCE_R3_PREDICT_BAND_CM ||
             error_cm * projected_error_cm <= 0.0f)) {
            s_balance.r3_phase =
                BALL_BALANCE_R3_PHASE_PREDICTIVE_BRAKE;
            s_balance.r3_brake_reverse_ms = 0U;
            s_balance.r3_unload_ms = 0U;
            s_balance.r3_brake_controller_sign =
                (velocity_cm_s > 0.0f) ? -1 : 1;

            brake_angle_deg =
                BALL_BALANCE_R3_BRAKE_ATTACK_ANGLE_DEG;
            if (brake_angle_deg > configured_max_angle_deg) {
                brake_angle_deg = configured_max_angle_deg;
            }

            *controller_output =
                (float)s_balance.r3_brake_controller_sign *
                brake_angle_deg;
            *braking_command = 1U;
        }

        return 1U;
    }

    if (s_balance.r3_phase ==
            BALL_BALANCE_R3_PHASE_PREDICTIVE_BRAKE) {
        if (s_balance.r3_brake_controller_sign == 0) {
            s_balance.r3_brake_controller_sign =
                (velocity_cm_s > 0.0f) ? -1 : 1;
        }

        /*
         * Measure how much brake the mechanism has actually built.  This is
         * crucial for a slow lead-screw actuator: command full travel only
         * while crossing from the previous drive side, then hold a milder
         * 0.55 degree brake once the measured flap reaches 0.32 degree.
         */
        desired_physical_brake_sign =
            (float)s_balance.config.control_sign *
            (float)s_balance.r3_brake_controller_sign;
        actual_brake_progress_deg =
            desired_physical_brake_sign *
            motor_current_flap_deg;
        if (actual_brake_progress_deg < 0.0f) {
            actual_brake_progress_deg = 0.0f;
        }

        brake_angle_deg =
            BALL_BALANCE_R3_BRAKE_ATTACK_ANGLE_DEG;
        if (actual_brake_progress_deg >=
                BALL_BALANCE_R3_BRAKE_ACTUAL_SWITCH_DEG) {
            brake_angle_deg =
                BALL_BALANCE_R3_BRAKE_HOLD_ANGLE_DEG;
        }
        if (brake_angle_deg > configured_max_angle_deg) {
            brake_angle_deg = configured_max_angle_deg;
        }

        /*
         * Once the ball is close and slow, taper the hold brake with speed.
         * The full attack is never tapered before the actuator has crossed to
         * the braking side, otherwise the screw would fail to build braking
         * angle in time.
         */
        brake_taper_ratio = 1.0f;
        if (actual_brake_progress_deg >=
                BALL_BALANCE_R3_BRAKE_ACTUAL_SWITCH_DEG &&
            ((float)s_balance.r3_target_direction_sign * error_cm < 0.0f ||
             BallBalance_Abs(error_cm) <=
                 BALL_BALANCE_R3_BRAKE_TAPER_ERROR_CM) &&
            BallBalance_Abs(velocity_cm_s) <
                BALL_BALANCE_R3_BRAKE_TAPER_SPEED_CM_S) {
            brake_taper_ratio =
                BallBalance_Abs(velocity_cm_s) /
                BALL_BALANCE_R3_BRAKE_TAPER_SPEED_CM_S;
            brake_taper_ratio = BallBalance_Limit(
                brake_taper_ratio,
                BALL_BALANCE_R3_BRAKE_TAPER_MIN_RATIO,
                1.0f);
        }

        /*
         * Release as soon as the velocity has genuinely reversed, or while
         * already inside the final 0.90 cm and below 1.35 cm/s.  This removes
         * the long full-angle tail that caused the light rebound.
         */
        release_brake = 0U;
        if ((float)s_balance.r3_target_direction_sign *
                    velocity_cm_s <=
                -BALL_BALANCE_R3_BRAKE_REVERSE_MIN_CM_S ||
            (BallBalance_Abs(error_cm) <=
                 BALL_BALANCE_R3_BRAKE_RELEASE_ERROR_CM &&
             BallBalance_Abs(velocity_cm_s) <=
                 BALL_BALANCE_R3_BRAKE_RELEASE_START_CM_S)) {
            release_brake = 1U;
        }

        if (release_brake != 0U) {
            if (s_balance.r3_brake_reverse_ms <
                    BALL_BALANCE_R3_BRAKE_RELEASE_MS) {
                s_balance.r3_brake_reverse_ms += elapsed_ms;
                if (s_balance.r3_brake_reverse_ms >
                        BALL_BALANCE_R3_BRAKE_RELEASE_MS) {
                    s_balance.r3_brake_reverse_ms =
                        BALL_BALANCE_R3_BRAKE_RELEASE_MS;
                }
            }
        } else if (moving_toward > 0.5f) {
            s_balance.r3_brake_reverse_ms = 0U;
        }

        release_ratio =
            1.0f -
            (float)s_balance.r3_brake_reverse_ms /
                (float)BALL_BALANCE_R3_BRAKE_RELEASE_MS;
        release_ratio =
            BallBalance_Limit(release_ratio, 0.0f, 1.0f);

        *controller_output =
            (float)s_balance.r3_brake_controller_sign *
            brake_angle_deg *
            brake_taper_ratio *
            release_ratio;
        *braking_command = 1U;

        if ((BallBalance_Abs(velocity_cm_s) <=
                 BALL_BALANCE_R3_BRAKE_STOP_SPEED_CM_S &&
             BallBalance_Abs(error_cm) <=
                 BALL_BALANCE_R3_FINE_ENTRY_ERROR_CM) ||
            s_balance.r3_brake_reverse_ms >=
                 BALL_BALANCE_R3_BRAKE_RELEASE_MS) {
            s_balance.r3_phase =
                BALL_BALANCE_R3_PHASE_BRAKE_UNLOAD;
            s_balance.r3_unload_ms = 0U;
            *controller_output = 0.0f;
            *braking_command = 0U;
        }

        return 1U;
    }

    if (s_balance.r3_phase ==
            BALL_BALANCE_R3_PHASE_BRAKE_UNLOAD) {
        if (s_balance.r3_unload_ms <
                BALL_BALANCE_R3_UNLOAD_MAX_MS) {
            s_balance.r3_unload_ms += elapsed_ms;
            if (s_balance.r3_unload_ms >
                    BALL_BALANCE_R3_UNLOAD_MAX_MS) {
                s_balance.r3_unload_ms =
                    BALL_BALANCE_R3_UNLOAD_MAX_MS;
            }
        }

        *controller_output = 0.0f;
        *braking_command = 0U;

        if ((s_balance.r3_unload_ms >=
                 BALL_BALANCE_R3_UNLOAD_MIN_MS &&
             BallBalance_Abs(motor_current_flap_deg) <=
                 BALL_BALANCE_R3_UNLOAD_MOTOR_WINDOW_DEG) ||
            s_balance.r3_unload_ms >=
                 BALL_BALANCE_R3_UNLOAD_MAX_MS) {
            s_balance.r3_phase =
                BALL_BALANCE_R3_PHASE_FINE_SETTLE;
        }

        return 1U;
    }

    /*
     * Fine-settle phase uses the normal position/velocity controller, but the
     * output angle is limited later to avoid another large-amplitude cycle.
     */
    return 0U;
}

static void BallBalance_ResetController(void)
{
    s_balance.previous_measurement_ms = 0U;
    s_balance.previous_ball_cm = 0.0f;
    s_balance.filtered_velocity_cm_s = 0.0f;
    s_balance.integral_error_cm_s = 0.0f;

    s_balance.breakaway_still_ms = 0U;
    s_balance.breakaway_elapsed_ms = 0U;
    s_balance.breakaway_cooldown_ms = 0U;
    s_balance.breakaway_start_ball_cm = 0.0f;
    s_balance.breakaway_active = 0U;
    s_balance.breakaway_soft = 0U;

    s_balance.settle_start_ms = 0U;
    s_balance.settle_timer_active = 0U;
    s_balance.settled = 0U;

    BallBalance_ResetRequirement3Motion();
}

/*
 * Update the one-shot breakaway state machine.
 *
 * Return 1 only while the larger starting angle must be applied. A real
 * displacement, rather than a single velocity sample, ends the boost so
 * camera-speed spikes cannot cancel it too early.
 */
static uint8_t BallBalance_UpdateBreakaway(
    float error_cm,
    float velocity_cm_s,
    float ball_cm,
    uint32_t elapsed_ms)
{
    float moved_cm;
    uint32_t active_max_ms;

    if (elapsed_ms > BALL_BALANCE_CAMERA_TIMEOUT_DEFAULT_MS) {
        elapsed_ms = BALL_BALANCE_PERIOD_MS;
    }

    if (s_balance.breakaway_cooldown_ms > elapsed_ms) {
        s_balance.breakaway_cooldown_ms -= elapsed_ms;
    } else {
        s_balance.breakaway_cooldown_ms = 0U;
    }

    /*
     * Once the ball is already inside the configured settling band, do not
     * fire another static-friction pulse.  This prevents the repeated
     * 0.70-degree pulses seen after the -5 cm position had already passed.
     */
    if (BallBalance_Abs(error_cm) <=
            s_balance.config.settle_position_error_cm) {
        s_balance.breakaway_active = 0U;
        s_balance.breakaway_still_ms = 0U;
        s_balance.breakaway_elapsed_ms = 0U;
        s_balance.breakaway_soft = 0U;
        return 0U;
    }

    if (s_balance.breakaway_active != 0U) {
        s_balance.breakaway_elapsed_ms += elapsed_ms;
        moved_cm = BallBalance_Abs(
            ball_cm - s_balance.breakaway_start_ball_cm);
        active_max_ms =
            (s_balance.breakaway_soft != 0U) ?
                BALL_BALANCE_BREAKAWAY_SOFT_MAX_MS :
                BALL_BALANCE_BREAKAWAY_FAR_MAX_MS;

        if (moved_cm >= BALL_BALANCE_BREAKAWAY_EXIT_MOVE_CM ||
            s_balance.breakaway_elapsed_ms >= active_max_ms) {
            s_balance.breakaway_active = 0U;
            s_balance.breakaway_soft = 0U;
            s_balance.breakaway_still_ms = 0U;
            s_balance.breakaway_elapsed_ms = 0U;
            s_balance.breakaway_cooldown_ms =
                BALL_BALANCE_BREAKAWAY_COOLDOWN_MS;
            return 0U;
        }

        return 1U;
    }

    if (s_balance.breakaway_cooldown_ms != 0U) {
        s_balance.breakaway_still_ms = 0U;
        return 0U;
    }

    if (BallBalance_Abs(velocity_cm_s) <=
            BALL_BALANCE_BREAKAWAY_STILL_SPEED_CM_S) {
        if (s_balance.breakaway_still_ms <
                BALL_BALANCE_BREAKAWAY_CONFIRM_MS) {
            s_balance.breakaway_still_ms += elapsed_ms;
        }

        if (s_balance.breakaway_still_ms >=
                BALL_BALANCE_BREAKAWAY_CONFIRM_MS) {
            s_balance.breakaway_active = 1U;
            s_balance.breakaway_soft =
                (BallBalance_Abs(error_cm) <=
                    BALL_BALANCE_BREAKAWAY_SOFT_MAX_ERROR_CM) ?
                    1U : 0U;
            s_balance.breakaway_elapsed_ms = 0U;
            s_balance.breakaway_still_ms = 0U;
            s_balance.breakaway_start_ball_cm = ball_cm;
            return 1U;
        }
    } else {
        s_balance.breakaway_still_ms = 0U;
    }

    return 0U;
}

static BallBalanceFault_t BallBalance_MapMotorFault(
    CL_Fault_t fault)
{
    switch (fault) {
        case CL_FAULT_NONE:
            return BALL_BALANCE_FAULT_NONE;

        case CL_FAULT_NO_ENCODER:
            return BALL_BALANCE_FAULT_MOTOR_ENCODER;

        case CL_FAULT_DIRECTION:
            return BALL_BALANCE_FAULT_MOTOR_DIRECTION;

        case CL_FAULT_DRIVER:
        default:
            return BALL_BALANCE_FAULT_MOTOR_DRIVER;
    }
}

static uint8_t BallBalance_ConfigValid(
    const BallBalanceConfig_t *config)
{
    if (config == 0) {
        return 0U;
    }

    if (!BallBalance_IsFinite(config->kp) ||
        !BallBalance_IsFinite(config->ki) ||
        !BallBalance_IsFinite(config->kd) ||
        !BallBalance_IsFinite(config->lever_arm_mm) ||
        !BallBalance_IsFinite(config->motor_deg_per_mm)) {
        return 0U;
    }

    if (config->kp < 0.0f ||
        config->ki < 0.0f ||
        config->kd < 0.0f) {
        return 0U;
    }

    if (config->control_sign != 1 &&
        config->control_sign != -1) {
        return 0U;
    }

    if (config->lever_arm_mm <= 0.0f ||
        config->motor_deg_per_mm <= 0.0f) {
        return 0U;
    }

    if (config->ball_target_min_cm >=
            config->ball_target_max_cm ||
        config->ball_safety_min_cm >=
            config->ball_safety_max_cm ||
        config->actuator_min_mm >=
            config->actuator_max_mm) {
        return 0U;
    }

    if (config->ball_target_min_cm <
            config->ball_safety_min_cm ||
        config->ball_target_max_cm >
            config->ball_safety_max_cm) {
        return 0U;
    }

    if (config->max_flap_angle_deg <= 0.0f ||
        config->max_flap_angle_deg > 10.0f ||
        config->integral_output_limit_deg < 0.0f ||
        config->velocity_filter_alpha < 0.0f ||
        config->velocity_filter_alpha > 1.0f ||
        config->camera_timeout_ms < 20U ||
        config->settle_time_ms < 20U) {
        return 0U;
    }

    return 1U;
}

static void BallBalance_SetFault(
    BallBalanceFault_t fault)
{
    CL_Stop(MOTOR_AXIS_X);

    s_balance.enabled = 0U;
    s_balance.settled = 0U;
    s_balance.fault = fault;
    s_balance.state = BALL_BALANCE_STATE_FAULT;

    BallBalance_ResetController();
}

static float BallBalance_FlapAngleToLiftMm(
    float angle_deg)
{
    float angle_rad;

    angle_rad =
        angle_deg * BALL_BALANCE_DEG_TO_RAD;

    /*
     * ��ҳ�ƺ�ҳת����
     * ˿��̧���� = ���۳��� * sin(�ڸ˽Ƕ�)��
     */
    return s_balance.config.lever_arm_mm *
           sinf(angle_rad);
}

static float BallBalance_LiftMmToFlapAngleDeg(
    float lift_mm)
{
    float ratio;

    ratio =
        lift_mm / s_balance.config.lever_arm_mm;

    return asinf(BallBalance_Limit(
        ratio,
        -1.0f,
        1.0f)) * BALL_BALANCE_RAD_TO_DEG;
}

static void BallBalance_UpdateSettled(
    float error_cm,
    float velocity_cm_s,
    uint32_t now_ms)
{
    if (BallBalance_Abs(error_cm) <=
            s_balance.config.settle_position_error_cm &&
        BallBalance_Abs(velocity_cm_s) <=
            s_balance.config.settle_velocity_cm_s) {

        if (s_balance.settle_timer_active == 0U) {
            s_balance.settle_timer_active = 1U;
            s_balance.settle_start_ms = now_ms;
        }

        if ((uint32_t)(now_ms -
                s_balance.settle_start_ms) >=
            s_balance.config.settle_time_ms) {
            s_balance.settled = 1U;
            s_balance.state =
                BALL_BALANCE_STATE_SETTLED;
        }
    } else {
        s_balance.settle_timer_active = 0U;
        s_balance.settled = 0U;
        s_balance.state =
            BALL_BALANCE_STATE_RUNNING;
    }
}

void BallBalance_Init(void)
{
    s_balance.config.kp = 0.20f;
    s_balance.config.ki = 0.0f;
    s_balance.config.kd = 0.06f;

    /*
     * ����������˿���쳤ʹ�Ҷ�����ʱ��
     * ������Ĭ�Ͽ��Ʒ���Ϊ-1��
     */
    s_balance.config.control_sign = -1;

    /*
     * ���밴ʵ�ʳߴ��޸ģ�
     * ��ҳ���ĵ�˿�����õ����ĵľ��롣
     */
    s_balance.config.lever_arm_mm = 235.0f;

    s_balance.config.motor_deg_per_mm = 180.0f;

    s_balance.config.ball_target_min_cm = -10.0f;
    s_balance.config.ball_target_max_cm = 10.0f;

    s_balance.config.ball_safety_min_cm = -12.5f;
    s_balance.config.ball_safety_max_cm = 12.5f;

    s_balance.config.max_flap_angle_deg = 1.50f;

    /*
     * 1.5�ȡ�250mm����Լ��6.0mm̧����
     * ��ʼ��ȫ�г�����Ϊ����7mm��
     */
    s_balance.config.actuator_min_mm = -7.0f;
    s_balance.config.actuator_max_mm = 7.0f;

    s_balance.config.integral_output_limit_deg = 0.12f;
    s_balance.config.velocity_filter_alpha = 0.15f;

   
    s_balance.config.camera_timeout_ms =
    BALL_BALANCE_CAMERA_TIMEOUT_DEFAULT_MS;

    s_balance.config.settle_position_error_cm = 0.45f;
    s_balance.config.settle_velocity_cm_s = 0.30f;
    s_balance.config.settle_time_ms = 140U;

    s_balance.target_ball_cm = 0.0f;
    s_balance.enabled = 0U;
    s_balance.external_feedforward_angle_deg = 0.0f;
    s_balance.external_feedforward_acceleration_mps2 = 0.0f;
    s_balance.external_feedforward_enabled = 0U;
    s_balance.external_feedforward_valid = 0U;
    s_balance.requirement4_mode = 0U;
    Req4VisualControl_Init();

    /*
     * Do not redefine the current power-up position as horizontal. The user
     * must physically level the rod once and send the level command.
     */
    s_balance.level_calibrated = 0U;

    s_balance.measured_ball_cm = 0.0f;
    s_balance.measurement_timestamp_ms = 0U;
    s_balance.measurement_sequence = 0U;
    s_balance.processed_sequence = 0U;

    s_balance.fault = BALL_BALANCE_FAULT_NONE;
    s_balance.state = BALL_BALANCE_STATE_NOT_READY;

    BallBalance_ResetController();

    s_balance.snapshot.target_ball_cm = 0.0f;
    s_balance.snapshot.current_ball_cm = 0.0f;
    s_balance.snapshot.ball_error_cm = 0.0f;
    s_balance.snapshot.ball_velocity_cm_s = 0.0f;
    s_balance.snapshot.p_output_deg = 0.0f;
    s_balance.snapshot.i_output_deg = 0.0f;
    s_balance.snapshot.d_output_deg = 0.0f;
    s_balance.snapshot.visual_flap_target_deg = 0.0f;
    s_balance.snapshot.feedforward_angle_deg = 0.0f;
    s_balance.snapshot.feedforward_acceleration_mps2 = 0.0f;
    s_balance.snapshot.flap_target_deg = 0.0f;
    s_balance.snapshot.actuator_target_mm = 0.0f;
    s_balance.snapshot.actuator_current_mm = 0.0f;
    s_balance.snapshot.motor_target_deg = 0.0f;
    s_balance.snapshot.motor_current_deg = 0.0f;
    s_balance.snapshot.measurement_sequence = 0U;
    s_balance.snapshot.measurement_timestamp_ms = 0U;
    s_balance.snapshot.level_calibrated = 0U;
    s_balance.snapshot.enabled = 0U;
    s_balance.snapshot.settled = 0U;
    s_balance.snapshot.feedforward_enabled = 0U;
    s_balance.snapshot.feedforward_valid = 0U;
    s_balance.snapshot.requirement4_mode = 0U;
    s_balance.snapshot.req4_breakaway_active = 0U;
    s_balance.snapshot.req4_visual_saturated = 0U;
    s_balance.snapshot.req4_in_one_cm_band = 0U;
    s_balance.snapshot.req4_raw_velocity_cm_s = 0.0f;
    s_balance.snapshot.req4_filtered_velocity_cm_s = 0.0f;
    s_balance.snapshot.req4_predicted_error_cm = 0.0f;
    s_balance.snapshot.req4_breakaway_output_deg = 0.0f;
    s_balance.snapshot.state =
        BALL_BALANCE_STATE_NOT_READY;
    s_balance.snapshot.fault =
        BALL_BALANCE_FAULT_NONE;
}

uint8_t BallBalance_PushBallPosition(
    float position_cm,
    uint32_t timestamp_ms)
{
    uint32_t primask;

    if (!BallBalance_IsFinite(position_cm)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    s_balance.measured_ball_cm = position_cm;
    s_balance.measurement_timestamp_ms = timestamp_ms;
    s_balance.measurement_sequence++;

    if (primask == 0U) {
        __enable_irq();
    }

    return 1U;
}
static void BallBalance_ProcessReturningZero(
    uint32_t now_ms)
{
    uint32_t measurement_ms;
    uint32_t sequence;

    /*
     * CL_Process() runs every 5 ms and moves the motor toward
     * the zero position recorded during initialization.
     */
    if (CL_IsReached(MOTOR_AXIS_X) == 0U) {
        return;
    }

    /*
     * The flap is now at its initialization zero position.
     * Wait for a fresh camera measurement before enabling
     * ball control. Waiting here does not generate a timeout fault.
     */
    BallBalance_ReadMeasurement(
        0,
        &measurement_ms,
        &sequence);

    if (sequence == 0U) {
        return;
    }

    if ((uint32_t)(now_ms - measurement_ms) >
        s_balance.config.camera_timeout_ms) {
        return;
    }

    s_balance.processed_sequence = 0U;
    s_balance.enabled = 1U;
    s_balance.state = BALL_BALANCE_STATE_RUNNING;

    BallBalance_ResetController();
}
void BallBalance_Process20ms(void)
{
    BallBalanceConfig_t config;
    BallBalanceFault_t motor_fault;
    CL_Fault_t cl_fault;

    uint32_t sequence;
    uint32_t measurement_ms;
    uint32_t now_ms;
    uint32_t delta_ms;

    float ball_cm;
    float target_cm;
    float error_cm;
    float raw_velocity;
    float velocity_cm_s;

    float p_output;
    float i_output;
    float d_output;
    float base_d_output;
    float speed_profile_output;
    float desired_velocity_cm_s;
    float desired_speed_cm_s;
    float predicted_displacement_cm;
    float predicted_error_cm;
    float brake_lead_time_s;
    float brake_lead_max_cm;
    float speed_profile_gain;
    float speed_profile_max_cm_s;
    float speed_track_gain;
    float stopping_distance_cm;
    float positive_transit_min_drive_deg;
    float integral_candidate;

    float controller_output;
    float visual_flap_target_deg;
    float feedforward_angle_deg;
    float feedforward_acceleration_mps2;
    float flap_target_deg;
    float actuator_target_mm;
    float actuator_current_mm;
    float motor_target_deg;
    float motor_current_deg;
    float motor_current_flap_deg;
    float positive_preposition_projected_cm;
    float flap_limit_deg;
    float breakaway_angle_deg;
    float drive_limit_deg;
    float abs_error_cm;
    uint8_t breakaway_output_active;
    uint8_t braking_command;
    uint8_t moving_toward_target;
    uint8_t stopping_brake_active;
    uint8_t r3_motion_override;
    uint8_t positive_preposition_override;
    uint8_t feedforward_active;
    uint8_t requirement4_visual_active;
    Req4VisualSnapshot_t req4_visual_snapshot;

    config = s_balance.config;
    now_ms = (uint32_t)tick_ms;

    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        feedforward_angle_deg =
            s_balance.external_feedforward_angle_deg;
        feedforward_acceleration_mps2 =
            s_balance.external_feedforward_acceleration_mps2;
        feedforward_active =
            (s_balance.external_feedforward_enabled != 0U &&
             s_balance.external_feedforward_valid != 0U) ? 1U : 0U;
        if (primask == 0U) {
            __enable_irq();
        }
    }

    /* Requirement 3 always keeps the original visual-only controller. */
    if (s_balance.r3_phase != BALL_BALANCE_R3_PHASE_NONE) {
        feedforward_active = 0U;
    }

    requirement4_visual_active =
        (s_balance.requirement4_mode != 0U &&
         s_balance.r3_phase == BALL_BALANCE_R3_PHASE_NONE &&
         Req4VisualControl_IsEnabled() != 0U) ? 1U : 0U;
    memset(&req4_visual_snapshot, 0, sizeof(req4_visual_snapshot));

    cl_fault = CL_GetFault(MOTOR_AXIS_X);
    motor_fault = BallBalance_MapMotorFault(cl_fault);

    if (motor_fault != BALL_BALANCE_FAULT_NONE) {
        BallBalance_SetFault(motor_fault);
        return;
    }
    if (s_balance.state ==
        BALL_BALANCE_STATE_RETURNING_ZERO) {
    BallBalance_ProcessReturningZero(now_ms);
    return;
    }
    if (s_balance.enabled == 0U) {
        return;
    }

    if (s_balance.level_calibrated == 0U) {
        BallBalance_Stop();
        s_balance.state = BALL_BALANCE_STATE_NOT_READY;
        return;
    }

    /*
    * UART2 may update these three values in an interrupt.
    * Read them together so they always belong to the same frame.
    */
    BallBalance_ReadMeasurement(
        &ball_cm,
        &measurement_ms,
        &sequence);

    if (sequence == 0U ||
        (uint32_t)(now_ms - measurement_ms) >
            config.camera_timeout_ms) {
        BallBalance_SetFault(
            BALL_BALANCE_FAULT_CAMERA_TIMEOUT);
        return;
    }

    /*
     * No new camera frame. Requirement 3 keeps the previous motor target.
     * Requirement 4 refreshes only the acceleration feed-forward term, so the
     * screw can react before the next camera frame without re-integrating old
     * visual data.
     */
    if (sequence == s_balance.processed_sequence) {
        if (requirement4_visual_active != 0U) {
            float refresh_visual =
                s_balance.snapshot.visual_flap_target_deg;
            float refresh_total = refresh_visual;
            float refresh_lift_mm;
            float refresh_motor_deg;

            if (feedforward_active != 0U) {
                refresh_total += feedforward_angle_deg;
                refresh_total = BallBalance_Limit(
                    refresh_total,
                    -config.max_flap_angle_deg,
                    config.max_flap_angle_deg);
            } else {
                feedforward_angle_deg = 0.0f;
                feedforward_acceleration_mps2 = 0.0f;
            }

            refresh_lift_mm = BallBalance_FlapAngleToLiftMm(refresh_total);
            refresh_motor_deg = refresh_lift_mm * config.motor_deg_per_mm;
            if (CL_SetTargetAngle(MOTOR_AXIS_X, refresh_motor_deg) != MOTOR_OK) {
                BallBalance_SetFault(BALL_BALANCE_FAULT_MOTOR_DRIVER);
                return;
            }

            s_balance.snapshot.feedforward_angle_deg =
                feedforward_angle_deg;
            s_balance.snapshot.feedforward_acceleration_mps2 =
                feedforward_acceleration_mps2;
            s_balance.snapshot.feedforward_valid =
                (feedforward_active != 0U) ? 1U : 0U;
            s_balance.snapshot.flap_target_deg = refresh_total;
            s_balance.snapshot.actuator_target_mm = refresh_lift_mm;
            s_balance.snapshot.motor_target_deg = refresh_motor_deg;
            s_balance.snapshot.motor_current_deg =
                CL_GetCurrentAngle(MOTOR_AXIS_X);
        }
        return;
    }

    s_balance.processed_sequence = sequence;

    if (ball_cm < config.ball_safety_min_cm ||
        ball_cm > config.ball_safety_max_cm) {
        BallBalance_SetFault(
            BALL_BALANCE_FAULT_BALL_OUT_OF_RANGE);
        return;
    }

    delta_ms = BALL_BALANCE_PERIOD_MS;

    if (s_balance.previous_measurement_ms == 0U) {
        raw_velocity = 0.0f;
        s_balance.filtered_velocity_cm_s = 0.0f;
    } else {
        delta_ms = measurement_ms -
                   s_balance.previous_measurement_ms;

        if (delta_ms < 5U) {
            delta_ms = 5U;
        }

        if (delta_ms > config.camera_timeout_ms) {
            delta_ms = config.camera_timeout_ms;
        }

        raw_velocity =
            (ball_cm - s_balance.previous_ball_cm) *
            1000.0f / (float)delta_ms;

        s_balance.filtered_velocity_cm_s +=
            config.velocity_filter_alpha *
            (raw_velocity -
             s_balance.filtered_velocity_cm_s);
    }

    s_balance.previous_ball_cm = ball_cm;
    s_balance.previous_measurement_ms = measurement_ms;

    velocity_cm_s =
        s_balance.filtered_velocity_cm_s;

    target_cm = BallBalance_Limit(
        s_balance.target_ball_cm,
        config.ball_target_min_cm,
        config.ball_target_max_cm);

    error_cm = target_cm - ball_cm;
    abs_error_cm = BallBalance_Abs(error_cm);

    /*
     * Read the measured mechanism angle before requirement-3 shaping.  The
     * two-stage brake uses this feedback to stop commanding full travel once
     * the lead screw has actually built enough braking tilt.
     */
    motor_current_deg =
        CL_GetCurrentAngle(MOTOR_AXIS_X);
    actuator_current_mm =
        motor_current_deg /
        config.motor_deg_per_mm;
    motor_current_flap_deg =
        BallBalance_LiftMmToFlapAngleDeg(
            actuator_current_mm);

    /*
     * Use very little prediction toward +5 cm because that point is crossed
     * and immediately followed by a reversal.  Keep the longer prediction on
     * the -5 cm leg, where final settling is required.
     */
    if (target_cm >= 0.0f) {
        brake_lead_time_s = BALL_BALANCE_POS_BRAKE_LEAD_TIME_S;
        brake_lead_max_cm = BALL_BALANCE_POS_BRAKE_LEAD_MAX_CM;
        speed_profile_gain = BALL_BALANCE_POS_SPEED_PROFILE_GAIN;
        speed_profile_max_cm_s = BALL_BALANCE_POS_SPEED_PROFILE_MAX_CM_S;
        speed_track_gain = BALL_BALANCE_POS_SPEED_TRACK_GAIN;
    } else {
        brake_lead_time_s = BALL_BALANCE_NEG_BRAKE_LEAD_TIME_S;
        brake_lead_max_cm = BALL_BALANCE_NEG_BRAKE_LEAD_MAX_CM;
        speed_profile_gain = BALL_BALANCE_NEG_SPEED_PROFILE_GAIN;
        speed_profile_max_cm_s = BALL_BALANCE_NEG_SPEED_PROFILE_MAX_CM_S;
        speed_track_gain = BALL_BALANCE_NEG_SPEED_TRACK_GAIN;
    }

    predicted_displacement_cm = velocity_cm_s * brake_lead_time_s;
    predicted_displacement_cm = BallBalance_Limit(
        predicted_displacement_cm,
        -brake_lead_max_cm,
        brake_lead_max_cm);
    predicted_error_cm =
        target_cm - (ball_cm + predicted_displacement_cm);

    p_output = config.kp * predicted_error_cm;

    desired_speed_cm_s =
        speed_profile_gain * BallBalance_Abs(predicted_error_cm);
    desired_speed_cm_s = BallBalance_Limit(
        desired_speed_cm_s,
        0.0f,
        speed_profile_max_cm_s);

    if (abs_error_cm <= BALL_BALANCE_TARGET_DEADBAND_CM) {
        desired_speed_cm_s = 0.0f;
    }

    desired_velocity_cm_s =
        BallBalance_Sign(predicted_error_cm) *
        desired_speed_cm_s;

    base_d_output = -config.kd * velocity_cm_s;
    speed_profile_output =
        speed_track_gain *
        (desired_velocity_cm_s - velocity_cm_s);

    /* Keep the public D snapshot useful by reporting all velocity shaping. */
    d_output = base_d_output + speed_profile_output;
    d_output = BallBalance_Limit(
        d_output,
        -BALL_BALANCE_VELOCITY_TERM_LIMIT_DEG,
        BALL_BALANCE_VELOCITY_TERM_LIMIT_DEG);

    integral_candidate =
        s_balance.integral_error_cm_s +
        error_cm * BALL_BALANCE_PERIOD_S;

    i_output = config.ki * integral_candidate;

    i_output = BallBalance_Limit(
        i_output,
        -config.integral_output_limit_deg,
        config.integral_output_limit_deg);

    if (config.ki > 0.0f) {
        integral_candidate = i_output / config.ki;
    } else {
        integral_candidate = 0.0f;
        i_output = 0.0f;
    }

    controller_output =
        p_output + i_output + d_output;

    /*
     * Ball-center stopping-distance brake.  controller_output has the same
     * sign as the desired ball acceleration before control_sign is applied.
     */
    moving_toward_target =
        (error_cm * velocity_cm_s > 0.0f) ? 1U : 0U;
    stopping_brake_active = 0U;
    stopping_distance_cm = 0.0f;

    if (s_balance.r3_phase == BALL_BALANCE_R3_PHASE_NONE &&
        moving_toward_target != 0U &&
        BallBalance_Abs(velocity_cm_s) >=
            BALL_BALANCE_STOP_MIN_SPEED_CM_S) {
        stopping_distance_cm =
            (velocity_cm_s * velocity_cm_s) /
                (2.0f * BALL_BALANCE_STOP_DECEL_CM_S2) +
            BallBalance_Abs(velocity_cm_s) *
                BALL_BALANCE_STOP_ACTUATOR_DELAY_S;

        if (stopping_distance_cm + BALL_BALANCE_STOP_MARGIN_CM >=
                abs_error_cm) {
            controller_output =
                -BallBalance_Sign(velocity_cm_s) *
                config.max_flap_angle_deg;
            stopping_brake_active = 1U;
        }
    }

    /*
     * Requirement-3 +5 -> -5 velocity prediction and acceleration shaping.
     * This override is active only after a live target change from + to -.
     */
    r3_motion_override =
        BallBalance_UpdateRequirement3Motion(
            target_cm,
            ball_cm,
            error_cm,
            velocity_cm_s,
            delta_ms,
            config.max_flap_angle_deg,
            motor_current_flap_deg,
            &controller_output,
            &stopping_brake_active);

    /*
     * Requirement-3 +5 cm actuator pre-positioning.
     *
     * Keep the successful 1.18 degree launch.  Once the forward prediction
     * reaches the +5 cm neighbourhood, latch a level command rather than a
     * reverse brake.  This removes most of the screw travel before the target
     * switches to -5 cm without adding a premature reverse acceleration.
     */
    positive_preposition_override = 0U;
    if (target_cm > 0.0f &&
        error_cm > 0.0f &&
        ball_cm < BALL_BALANCE_POS_TRANSIT_SWITCH_MIN_CM) {
        positive_preposition_projected_cm =
            ball_cm +
            ((velocity_cm_s > 0.0f) ? velocity_cm_s : 0.0f) *
                BALL_BALANCE_POS_PREPOSITION_PREDICT_TIME_S;

        if (s_balance.positive_preposition_active == 0U &&
            ball_cm >= BALL_BALANCE_POS_PREPOSITION_MIN_CENTER_CM &&
            velocity_cm_s >= BALL_BALANCE_POS_PREPOSITION_MIN_SPEED_CM_S &&
            positive_preposition_projected_cm >=
                BALL_BALANCE_POS_PREPOSITION_TRIGGER_CM) {
            s_balance.positive_preposition_active = 1U;
        }

        if (s_balance.positive_preposition_active != 0U) {
            if (velocity_cm_s <=
                    BALL_BALANCE_POS_PREPOSITION_CREEP_SPEED_CM_S) {
                /*
                 * If early leveling removes too much speed, retain a small
                 * forward creep so the BALL CENTER still enters 4.55..5.45 cm.
                 */
                controller_output =
                    BALL_BALANCE_POS_PREPOSITION_CREEP_DRIVE_DEG;
                stopping_brake_active = 0U;
            } else if (ball_cm >=
                    BALL_BALANCE_POS_PREPOSITION_BRAKE_START_CM) {
                /*
                 * In the final approach, use only a very small reverse tilt.
                 * Its main purpose is to move the slow screw just across the
                 * mechanical zero before the +5 -> -5 target switch.  The
                 * angle is intentionally much smaller than the old 0.32 deg
                 * pre-brake, so it should not stop the ball before 4.55 cm.
                 */
                controller_output =
                    -BALL_BALANCE_POS_PREPOSITION_BRAKE_DEG;
                stopping_brake_active = 1U;
            } else {
                controller_output = 0.0f;
                stopping_brake_active = 0U;
            }
            positive_preposition_override = 1U;
        } else {
            positive_transit_min_drive_deg =
                BallBalance_GetPositiveTransitMinDrive(
                    ball_cm,
                    BALL_BALANCE_POSITIVE_DRIVE_LIMIT_DEG);

            if (controller_output < positive_transit_min_drive_deg) {
                controller_output = positive_transit_min_drive_deg;
            }
            stopping_brake_active = 0U;
        }
    }

    if (r3_motion_override != 0U ||
        positive_preposition_override != 0U) {
        s_balance.breakaway_active = 0U;
        s_balance.breakaway_still_ms = 0U;
        s_balance.breakaway_elapsed_ms = 0U;
        breakaway_output_active = 0U;
    } else {
        breakaway_output_active =
            BallBalance_UpdateBreakaway(
                error_cm,
                velocity_cm_s,
                ball_cm,
                delta_ms);
    }

    if (breakaway_output_active != 0U) {
        /*
         * Short direction-dependent static-friction pulse.
         */
        breakaway_angle_deg =
            BallBalance_GetBreakawayAngle(
                error_cm,
                s_balance.breakaway_soft);
        controller_output =
            (error_cm >= 0.0f) ?
                breakaway_angle_deg :
                -breakaway_angle_deg;
        flap_limit_deg = BallBalance_Limit(
            breakaway_angle_deg,
            0.0f,
            config.max_flap_angle_deg);
    } else {
        /*
         * A command with the opposite sign to the real position error is a
         * braking command. Braking may use the full maxangle, including near
         * the target. Only drive commands are tapered and direction limited.
         */
        braking_command = stopping_brake_active;
        if (BallBalance_Sign(controller_output) != 0.0f &&
            BallBalance_Sign(error_cm) != 0.0f &&
            BallBalance_Sign(controller_output) !=
                BallBalance_Sign(error_cm)) {
            braking_command = 1U;
        }

        if (braking_command != 0U) {
            flap_limit_deg = config.max_flap_angle_deg;
        } else {
            if (error_cm >= 0.0f) {
                drive_limit_deg =
                    BALL_BALANCE_POSITIVE_DRIVE_LIMIT_DEG;
            } else {
                drive_limit_deg =
                    BALL_BALANCE_NEGATIVE_DRIVE_LIMIT_DEG;
            }

            if (drive_limit_deg > config.max_flap_angle_deg) {
                drive_limit_deg = config.max_flap_angle_deg;
            }

            flap_limit_deg =
                BallBalance_GetApproachAngleLimit(
                    drive_limit_deg,
                    abs_error_cm);
        }
    }

    if (s_balance.r3_phase ==
            BALL_BALANCE_R3_PHASE_FINE_SETTLE) {
        if (breakaway_output_active != 0U) {
            if (flap_limit_deg >
                    BALL_BALANCE_R3_FINE_BREAKAWAY_MAX_DEG) {
                flap_limit_deg =
                    BALL_BALANCE_R3_FINE_BREAKAWAY_MAX_DEG;
            }
        } else if (flap_limit_deg >
                BALL_BALANCE_R3_SETTLE_MAX_ANGLE_DEG) {
            flap_limit_deg =
                BALL_BALANCE_R3_SETTLE_MAX_ANGLE_DEG;
        }
    }

    if (requirement4_visual_active != 0U) {
        /* Requirement 4 uses an independent visual controller. It reuses the
         * successful requirement-3 ideas: ball-center error, velocity damping,
         * short prediction and one-shot static-friction compensation. Its
         * tunable values never modify the requirement-3 configuration. */
        Req4VisualControl_Update(
            target_cm,
            ball_cm,
            raw_velocity,
            delta_ms,
            config.control_sign);
        Req4VisualControl_GetSnapshot(&req4_visual_snapshot);

        p_output = req4_visual_snapshot.p_output_deg;
        i_output = req4_visual_snapshot.i_output_deg;
        d_output = req4_visual_snapshot.d_output_deg;
        visual_flap_target_deg =
            req4_visual_snapshot.output_flap_deg;
        breakaway_output_active = 0U;
    } else {
        visual_flap_target_deg =
            (float)config.control_sign *
            controller_output;

        /*
         * Remove very small visual-feedback commands near the target to avoid a
         * camera-noise limit cycle. Requirement-4 acceleration feed-forward is
         * added afterwards, so it remains available even when visual error is zero.
         */
        if (BallBalance_Abs(error_cm) <=
                BALL_BALANCE_TARGET_DEADBAND_CM &&
            BallBalance_Abs(velocity_cm_s) <=
                BALL_BALANCE_BREAKAWAY_STILL_SPEED_CM_S) {
            visual_flap_target_deg = 0.0f;
        }

        visual_flap_target_deg = BallBalance_Limit(
            visual_flap_target_deg,
            -flap_limit_deg,
            flap_limit_deg);
    }

    flap_target_deg = visual_flap_target_deg;
    if (feedforward_active != 0U) {
        flap_target_deg += feedforward_angle_deg;
        flap_target_deg = BallBalance_Limit(
            flap_target_deg,
            -config.max_flap_angle_deg,
            config.max_flap_angle_deg);
    } else {
        feedforward_angle_deg = 0.0f;
        feedforward_acceleration_mps2 = 0.0f;
    }

    /*
     * �������δ���ͣ�����������ڰ����˳�����ʱ���֡�
     */
    if (requirement4_visual_active != 0U) {
        /* The requirement-4 module owns its integral state. Keep the original
         * requirement-3/general integral cleared while this mode is active. */
        s_balance.integral_error_cm_s = 0.0f;
    } else if (breakaway_output_active != 0U) {
        /* Freeze the integral while the one-shot static-friction pulse runs. */
        i_output =
            config.ki *
            s_balance.integral_error_cm_s;
    } else if (BallBalance_Abs(
            (float)config.control_sign *
            controller_output) <=
            flap_limit_deg ||
        (visual_flap_target_deg >= flap_limit_deg &&
         (float)config.control_sign * error_cm < 0.0f) ||
        (visual_flap_target_deg <= -flap_limit_deg &&
         (float)config.control_sign * error_cm > 0.0f)) {
        s_balance.integral_error_cm_s =
            integral_candidate;
    } else {
        i_output =
            config.ki *
            s_balance.integral_error_cm_s;
    }

    actuator_target_mm =
        BallBalance_FlapAngleToLiftMm(
            flap_target_deg);

    if (actuator_target_mm <
            config.actuator_min_mm ||
        actuator_target_mm >
            config.actuator_max_mm) {
        BallBalance_SetFault(
            BALL_BALANCE_FAULT_ACTUATOR_OVER_TRAVEL);
        return;
    }

    motor_target_deg =
        actuator_target_mm *
        config.motor_deg_per_mm;

    if (CL_SetTargetAngle(
            MOTOR_AXIS_X,
            motor_target_deg) != MOTOR_OK) {
        BallBalance_SetFault(
            BALL_BALANCE_FAULT_MOTOR_DRIVER);
        return;
    }

    /*
     * motor_current_deg and actuator_current_mm were sampled before motion
     * shaping so the same feedback is used for both control and telemetry.
     */
    BallBalance_UpdateSettled(
        error_cm,
        velocity_cm_s,
        now_ms);

    if (s_balance.settled == 0U) {
        s_balance.state =
            BALL_BALANCE_STATE_RUNNING;
    }

    s_balance.snapshot.target_ball_cm = target_cm;
    s_balance.snapshot.current_ball_cm = ball_cm;
    s_balance.snapshot.ball_error_cm = error_cm;
    s_balance.snapshot.ball_velocity_cm_s =
        velocity_cm_s;

    s_balance.snapshot.p_output_deg = p_output;
    s_balance.snapshot.i_output_deg = i_output;
    s_balance.snapshot.d_output_deg = d_output;
    s_balance.snapshot.visual_flap_target_deg =
        visual_flap_target_deg;
    s_balance.snapshot.feedforward_angle_deg =
        feedforward_angle_deg;
    s_balance.snapshot.feedforward_acceleration_mps2 =
        feedforward_acceleration_mps2;
    s_balance.snapshot.flap_target_deg =
        flap_target_deg;
    s_balance.snapshot.req4_raw_velocity_cm_s =
        req4_visual_snapshot.raw_velocity_cm_s;
    s_balance.snapshot.req4_filtered_velocity_cm_s =
        req4_visual_snapshot.filtered_velocity_cm_s;
    s_balance.snapshot.req4_predicted_error_cm =
        req4_visual_snapshot.predicted_error_cm;
    s_balance.snapshot.req4_breakaway_output_deg =
        req4_visual_snapshot.breakaway_output_deg;

    s_balance.snapshot.actuator_target_mm =
        actuator_target_mm;
    s_balance.snapshot.actuator_current_mm =
        actuator_current_mm;

    s_balance.snapshot.motor_target_deg =
        motor_target_deg;
    s_balance.snapshot.motor_current_deg =
        motor_current_deg;

    s_balance.snapshot.measurement_sequence =
        sequence;
    s_balance.snapshot.measurement_timestamp_ms =
        measurement_ms;

    s_balance.snapshot.level_calibrated =
        s_balance.level_calibrated;
    s_balance.snapshot.enabled =
        s_balance.enabled;
    s_balance.snapshot.settled =
        s_balance.settled;
    s_balance.snapshot.feedforward_enabled =
        s_balance.external_feedforward_enabled;
    s_balance.snapshot.feedforward_valid =
        (feedforward_active != 0U) ? 1U : 0U;
    s_balance.snapshot.requirement4_mode =
        s_balance.requirement4_mode;
    s_balance.snapshot.req4_breakaway_active =
        req4_visual_snapshot.breakaway_active;
    s_balance.snapshot.req4_visual_saturated =
        req4_visual_snapshot.saturated;
    s_balance.snapshot.req4_in_one_cm_band =
        req4_visual_snapshot.in_one_cm_band;
    s_balance.snapshot.state =
        s_balance.state;
    s_balance.snapshot.fault =
        s_balance.fault;
}

uint8_t BallBalance_Enable(uint8_t enable)
{
    if (enable == 0U) {
        BallBalance_Stop();
        return 1U;
    }

    if (s_balance.level_calibrated == 0U) {
        s_balance.enabled = 0U;
        s_balance.state = BALL_BALANCE_STATE_NOT_READY;
        return 0U;
    }

    /*
     * Enabling is also the fault recovery command:
     *
     * 1. Stop the current balance command.
     * 2. Clear the motor and balance faults.
     * 3. Move to the zero recorded during initialization.
     * 4. Start ball control only after zero is reached.
     */
    BallBalance_Stop();
    CL_ClearFault(MOTOR_AXIS_X);

    s_balance.enabled = 0U;
    s_balance.settled = 0U;
    s_balance.fault = BALL_BALANCE_FAULT_NONE;

    BallBalance_ResetController();

    /*
     * This moves to the existing zero position.
     * Do not call CL_SetZero() here because that would redefine
     * the current fault position as zero.
     */
    if (CL_SetTargetAngle(
            MOTOR_AXIS_X,
            0.0f) != MOTOR_OK) {
        BallBalance_SetFault(
            BALL_BALANCE_FAULT_MOTOR_DRIVER);
        return 0U;
    }

    s_balance.state =
        BALL_BALANCE_STATE_RETURNING_ZERO;

    return 1U;
}
uint8_t BallBalance_SetTargetCm(float target_cm)
{
    float previous_target_cm;

    if (!BallBalance_IsFinite(target_cm) ||
        target_cm <
            s_balance.config.ball_target_min_cm ||
        target_cm >
            s_balance.config.ball_target_max_cm) {
        return 0U;
    }

    previous_target_cm = s_balance.target_ball_cm;
    s_balance.target_ball_cm = target_cm;

    /*
     * A live +5 cm -> -5 cm target change is the requirement-3 reversal.
     * Preserve the measured velocity and start the dedicated velocity-phase
     * controller. Other target changes use the normal controller.
     */
    if (s_balance.enabled != 0U &&
        previous_target_cm > 0.0f &&
        target_cm < 0.0f) {
        s_balance.r3_phase =
            BALL_BALANCE_R3_PHASE_REVERSE_DRIVE;
        s_balance.r3_toward_velocity_ms = 0U;
        s_balance.r3_brake_reverse_ms = 0U;
        s_balance.r3_unload_ms = 0U;
        s_balance.r3_stall_ms = 0U;
        s_balance.positive_preposition_active = 0U;
        s_balance.r3_target_direction_sign =
            (target_cm >= previous_target_cm) ? 1 : -1;
        s_balance.r3_brake_controller_sign = 0;
    } else {
        BallBalance_ResetRequirement3Motion();
    }

    s_balance.integral_error_cm_s = 0.0f;
    s_balance.breakaway_active = 0U;
    s_balance.breakaway_still_ms = 0U;
    s_balance.breakaway_elapsed_ms = 0U;
    s_balance.breakaway_cooldown_ms = 0U;
    s_balance.settled = 0U;
    s_balance.settle_timer_active = 0U;
    s_balance.settle_start_ms = 0U;

    return 1U;
}

void BallBalance_Stop(void)
{
    s_balance.enabled = 0U;
    s_balance.settled = 0U;
    s_balance.external_feedforward_enabled = 0U;
    s_balance.external_feedforward_valid = 0U;
    s_balance.external_feedforward_angle_deg = 0.0f;
    s_balance.external_feedforward_acceleration_mps2 = 0.0f;
    s_balance.requirement4_mode = 0U;
    Req4VisualControl_SetEnabled(0U);

    CL_Stop(MOTOR_AXIS_X);
    BallBalance_ResetController();

    if (s_balance.fault !=
            BALL_BALANCE_FAULT_NONE) {
        s_balance.state =
            BALL_BALANCE_STATE_FAULT;
    } else if (s_balance.level_calibrated == 0U) {
        s_balance.state =
            BALL_BALANCE_STATE_NOT_READY;
    } else {
        s_balance.state =
            BALL_BALANCE_STATE_DISABLED;
    }

    /*
     * CL_Stop() holds the mechanism at its current encoder position.  Refresh
     * the snapshot so plot=1 does not keep printing the last active target
     * after a TEST3 abort.
     */
    s_balance.snapshot.motor_current_deg =
        CL_GetCurrentAngle(MOTOR_AXIS_X);
    s_balance.snapshot.motor_target_deg =
        s_balance.snapshot.motor_current_deg;
    s_balance.snapshot.actuator_current_mm =
        s_balance.snapshot.motor_current_deg /
        s_balance.config.motor_deg_per_mm;
    s_balance.snapshot.actuator_target_mm =
        s_balance.snapshot.actuator_current_mm;
    s_balance.snapshot.flap_target_deg =
        asinf(BallBalance_Limit(
            s_balance.snapshot.actuator_current_mm /
                s_balance.config.lever_arm_mm,
            -1.0f,
            1.0f)) * BALL_BALANCE_RAD_TO_DEG;
    s_balance.snapshot.enabled = 0U;
    s_balance.snapshot.settled = 0U;
    s_balance.snapshot.feedforward_enabled = 0U;
    s_balance.snapshot.feedforward_valid = 0U;
    s_balance.snapshot.requirement4_mode = 0U;
    s_balance.snapshot.req4_breakaway_active = 0U;
    s_balance.snapshot.req4_visual_saturated = 0U;
    s_balance.snapshot.req4_in_one_cm_band = 0U;
    s_balance.snapshot.req4_raw_velocity_cm_s = 0.0f;
    s_balance.snapshot.req4_filtered_velocity_cm_s = 0.0f;
    s_balance.snapshot.req4_predicted_error_cm = 0.0f;
    s_balance.snapshot.req4_breakaway_output_deg = 0.0f;
    s_balance.snapshot.visual_flap_target_deg =
        s_balance.snapshot.flap_target_deg;
    s_balance.snapshot.feedforward_angle_deg = 0.0f;
    s_balance.snapshot.feedforward_acceleration_mps2 = 0.0f;
    s_balance.snapshot.state = s_balance.state;
    s_balance.snapshot.fault = s_balance.fault;
}

void BallBalance_SetLevelZero(void)
{
    BallBalance_Stop();

    /*
     * ����ǰ���뱣֤�ڸ��Ѿ��˹���ƽ��
     */
    CL_SetZero(MOTOR_AXIS_X);

    s_balance.target_ball_cm = 0.0f;
    s_balance.level_calibrated = 1U;
    s_balance.fault = BALL_BALANCE_FAULT_NONE;
    s_balance.state = BALL_BALANCE_STATE_DISABLED;

    BallBalance_ResetController();
}

void BallBalance_SetExternalFeedforwardEnabled(uint8_t enable)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_balance.external_feedforward_enabled =
        (enable != 0U) ? 1U : 0U;
    if (enable == 0U) {
        s_balance.external_feedforward_valid = 0U;
        s_balance.external_feedforward_angle_deg = 0.0f;
        s_balance.external_feedforward_acceleration_mps2 = 0.0f;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

void BallBalance_UpdateExternalFeedforward(
    float angle_deg,
    float acceleration_mps2,
    uint8_t valid)
{
    uint32_t primask;

    if (!BallBalance_IsFinite(angle_deg) ||
        !BallBalance_IsFinite(acceleration_mps2)) {
        valid = 0U;
        angle_deg = 0.0f;
        acceleration_mps2 = 0.0f;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    s_balance.external_feedforward_angle_deg = angle_deg;
    s_balance.external_feedforward_acceleration_mps2 =
        acceleration_mps2;
    s_balance.external_feedforward_valid =
        (valid != 0U) ? 1U : 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t BallBalance_IsExternalFeedforwardEnabled(void)
{
    return s_balance.external_feedforward_enabled;
}

void BallBalance_ClearFault(void)
{
    BallBalance_Stop();
    CL_ClearFault(MOTOR_AXIS_X);

    s_balance.fault = BALL_BALANCE_FAULT_NONE;

    if (s_balance.level_calibrated != 0U) {
        s_balance.state =
            BALL_BALANCE_STATE_DISABLED;
    } else {
        s_balance.state =
            BALL_BALANCE_STATE_NOT_READY;
    }
}

void BallBalance_SetRequirement4Mode(uint8_t enable)
{
    s_balance.requirement4_mode = (enable != 0U) ? 1U : 0U;
    Req4VisualControl_SetEnabled(enable);
    s_balance.snapshot.requirement4_mode = s_balance.requirement4_mode;
    s_balance.snapshot.req4_breakaway_active = 0U;
    s_balance.snapshot.req4_visual_saturated = 0U;
    s_balance.snapshot.req4_in_one_cm_band = 0U;
    s_balance.snapshot.req4_raw_velocity_cm_s = 0.0f;
    s_balance.snapshot.req4_filtered_velocity_cm_s = 0.0f;
    s_balance.snapshot.req4_predicted_error_cm = 0.0f;
    s_balance.snapshot.req4_breakaway_output_deg = 0.0f;
}

uint8_t BallBalance_IsRequirement4Mode(void)
{
    return s_balance.requirement4_mode;
}

uint8_t BallBalance_SetPid(
    float kp,
    float ki,
    float kd)
{
    BallBalanceConfig_t config;

    BallBalance_GetConfig(&config);

    config.kp = kp;
    config.ki = ki;
    config.kd = kd;

    return BallBalance_SetConfig(&config);
}

void BallBalance_GetConfig(
    BallBalanceConfig_t *config)
{
    uint32_t primask;

    if (config == 0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *config = s_balance.config;

    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t BallBalance_SetConfig(
    const BallBalanceConfig_t *config)
{
    uint32_t primask;

    if (BallBalance_ConfigValid(config) == 0U) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    s_balance.config = *config;
    BallBalance_ResetController();

    if (primask == 0U) {
        __enable_irq();
    }

    return 1U;
}

uint8_t BallBalance_IsReady(void)
{
    return s_balance.level_calibrated != 0U &&
           s_balance.measurement_sequence != 0U &&
           s_balance.fault ==
               BALL_BALANCE_FAULT_NONE;
}

uint8_t BallBalance_IsEnabled(void)
{
    return s_balance.enabled;
}

uint8_t BallBalance_IsSettled(void)
{
    return s_balance.settled;
}

BallBalanceState_t BallBalance_GetState(void)
{
    return s_balance.state;
}

BallBalanceFault_t BallBalance_GetFault(void)
{
    return s_balance.fault;
}

void BallBalance_GetSnapshot(
    BallBalanceSnapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == 0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *snapshot = s_balance.snapshot;

    /*
     * Camera UART reception continues while the controller is disabled.
     * The cached control snapshot is only refreshed by the active 20 ms
     * controller, so expose the live measurement here as well.  Without
     * this, requirement 6 sees measurement_sequence == 0 before startup,
     * cannot latch the current ball position, and the car times out while
     * waiting for the READY acknowledgement.
     */
    snapshot->current_ball_cm =
        s_balance.measured_ball_cm;
    snapshot->measurement_timestamp_ms =
        s_balance.measurement_timestamp_ms;
    snapshot->measurement_sequence =
        s_balance.measurement_sequence;

    snapshot->target_ball_cm =
        s_balance.target_ball_cm;
    snapshot->level_calibrated =
        s_balance.level_calibrated;
    snapshot->enabled =
        s_balance.enabled;
    snapshot->settled =
        s_balance.settled;
    snapshot->feedforward_enabled =
        s_balance.external_feedforward_enabled;
    snapshot->feedforward_valid =
        s_balance.external_feedforward_valid;
    snapshot->state =
        s_balance.state;
    snapshot->fault =
        s_balance.fault;

    if (primask == 0U) {
        __enable_irq();
    }
}
