#ifndef CONFIG_CAR_CONFIG_H_
#define CONFIG_CAR_CONFIG_H_

/*
 * Fast-finish oval-track configuration.
 *
 * Design goals:
 *   - run the simple 1.5 m + r=0.5 m oval without intersection logic;
 *   - detect the start/finish stripe with minimum latency;
 *   - keep following the last valid line state through a short sensor dropout;
 *   - actively brake if the line does not return within a bounded time.
 */

/* Speed PI and encoder remain at 20 ms. */
#define APP_CONTROL_PERIOD_MS                 (20U)

/* Line PD runs at 5 ms. */
#define LINE_CONTROL_PERIOD_MS                (5U)
#define LINE_D_REFERENCE_PERIOD_MS            (20U)

/* Start/finish and raw line supervision run faster than line PD. */
#define CAR_SUPERVISOR_PERIOD_MS              (3U)

/* ---------------- Speed PI ---------------- */
#define SPEED_PI_SCALE_Q12                    (4096L)
#define SPEED_PI_DEFAULT_M1_KP_Q12            (200L)
#define SPEED_PI_DEFAULT_M1_KI_Q12            (3L)
#define SPEED_PI_DEFAULT_M2_KP_Q12            (200L)
#define SPEED_PI_DEFAULT_M2_KI_Q12            (3L)
#define SPEED_PI_OUTPUT_LIMIT_PERCENT         (90)
#define SPEED_PI_TARGET_LIMIT_CPS             (4000L)
#define SPEED_PI_FILTER_DIVISOR               (4L)

/*
 * Feed-forward calibrated from the user's measurements:
 *   900 CPS  -> about 20% PWM
 *   2500 CPS -> about 47-48% PWM
 */
#define SPEED_PI_FEEDFORWARD_NUM              (19L)
#define SPEED_PI_FEEDFORWARD_DIV              (1000L)

#define SPEED_PI_STARTUP_EXIT_CPS             (80L)
#define SPEED_PI_STARTUP_MIN_PERCENT_MAX      (50U)

/* ---------------- High-speed line PD ------- */
#define LINE_DEFAULT_BASE_CPS                 (2600L)
#define LINE_MIN_BASE_CPS                     (100L)
#define LINE_MAX_BASE_CPS                     (3200L)
#define LINE_MAX_TARGET_CPS                   SPEED_PI_TARGET_LIMIT_CPS
#define LINE_MAX_CORRECTION_CPS               (1100L)

#define LINE_DEFAULT_KP_NUM                   (19L)
#define LINE_DEFAULT_KD_NUM                   (7L)
#define LINE_GAIN_DIV                         (100L)
#define LINE_KP_NUM_MIN                       (0L)
#define LINE_KP_NUM_MAX                       (100L)
#define LINE_KD_NUM_MIN                       (0L)
#define LINE_KD_NUM_MAX                       (100L)
#define LINE_STEER_SIGN                       (1L)

#define LINE_D_FILTER_SHIFT                   (1U)
#define LINE_DERIVATIVE_LIMIT                 (5000L)
#define LINE_CORRECTION_SLEW_CPS_PER_STEP     (250L)

/*
 * Temporary line-loss recovery.
 *
 * When all sensors lose the line, reuse the last valid steering correction
 * for up to 140 ms, but reduce the average speed. This is the requested
 * "return to the previous line-follow state" behavior. Persistent loss still
 * brakes the car instead of driving indefinitely with stale data.
 */
#define LINE_LOST_REPLAY_MS                   (140U)
#define LINE_LOST_REPLAY_BASE_CPS             (1500L)
#define LINE_LOST_REPLAY_MAX_CORRECTION_CPS   (900L)

/* Mild speed scheduling for the two r=0.5 m semicircles. */
#define LINE_SHARP_TURN_START_ERROR           (1700L)
#define LINE_SHARP_TURN_FULL_ERROR            (3500L)
#define LINE_SHARP_TURN_MIN_BASE_CPS          (2100L)

/* ---------------- Simplified race state ---- */
/*
 * Mission defaults selected by the three physical buttons.
 *
 * Requirement 2 keeps the existing fast-lap strategy unchanged.
 * Requirements 4/5/6 share one continuous line-follow program at 1500 CPS.
 * The continuous program does not stop at B, A, 8 s, 25 s or 30 s; it keeps
 * following until a button/serial stop command or a safety fault occurs.
 */
#define CAR_FAST_BASE_CPS                     (2600L)
#define CAR_CONTINUOUS_BASE_CPS               (1500L)
#define CAR_DEFAULT_BASE_CPS                  CAR_FAST_BASE_CPS
#define CAR_MIN_BASE_CPS                      LINE_MIN_BASE_CPS
#define CAR_MAX_BASE_CPS                      LINE_MAX_BASE_CPS

/*
 * Start/finish marker condition. A sample is treated as the A-point marker
 * when any six or more sensors are black, or when either required central
 * five-sensor pattern is present:
 *
 *   S2 S3 S4 S5 S6 -> mask 0x3E
 *
 * Mask bit mapping is bit0=S1 ... bit7=S8. S3..S7 (0x7C) is deliberately
 * excluded because it occurs during verified clockwise cornering and caused
 * false requirement-2 stops on the real vehicle.
 */
#define CAR_MARKER_MIN_TOTAL_BLACK_COUNT      (6U)
#define CAR_MARKER_CENTER5_S2_TO_S6_MASK      (0x3EU)

#define CAR_START_MARKER_STABLE_MS            (20U)
#define CAR_MARKER_LEAVE_STABLE_MS            (50U)
#define CAR_START_MARKER_TIMEOUT_MS           (0U)

/* Ignore wide patterns early in the lap. */
#define CAR_FINISH_MIN_DRIVE_MS               (16500U)

/*
 * Requirement 2 late-lap speed reduction and safety timeout.
 *
 * These values and the related control behavior are intentionally preserved
 * from the uploaded project. Requirements 4/5/6 do not use any timed speed
 * change or timed/marker stop.
 */
#define CAR_REQ2_SLOWDOWN_START_MS            (16500U)
#define CAR_REQ2_SLOWDOWN_CPS                 (1000L)
#define CAR_REQ2_NO_MARKER_STOP_MS            (20000U)

/*
 * CarControl permits the LineFollow replay window. If the line is still lost
 * after 180 ms, active brake and enter FAULT.
 */
#define CAR_LINE_LOST_FAULT_MS                (180U)

#define CAR_STOP_CPS                          (30L)
#define CAR_STOP_STABLE_MS                    (60U)
#define CAR_STOP_TIMEOUT_MS                   (500U)

/* Three active-low buttons on PB14/PB11/PB10, each connected to GND. */
#define START_BUTTON_TASK_PERIOD_MS           (5U)
#define START_BUTTON_DEBOUNCE_MS              (25U)

/* ---------------- VOFA serial tuning ------- */
#define VOFA_COMMAND_BUFFER_SIZE              (96U)
#define VOFA_DEFAULT_PLOT_PERIOD_MS           (100U)
#define VOFA_MIN_PLOT_PERIOD_MS               (20U)
#define VOFA_MAX_PLOT_PERIOD_MS               (1000U)
#define VOFA_MAX_MISSION_COUNT                (100U)

/* ---------------- OLED display ------------- */
#define CAR_ENABLE_OLED                       (1U)
#define OLED_I2C_ADDRESS_7BIT                 (0x3CU)
#define OLED_WIDTH                            (128U)
#define OLED_HEIGHT                           (64U)
#define OLED_PAGE_COUNT                       (OLED_HEIGHT / 8U)
#define OLED_COLUMN_OFFSET                    (0U)
#define OLED_TASK_PERIOD_MS                   (50U)
#define OLED_POWER_ON_DELAY_MS                (300U)
#define OLED_BOOT_RETRY_COUNT                 (3U)
#define OLED_BOOT_RETRY_GAP_MS                (60U)
#define OLED_RETRY_PERIOD_MS                  (1000U)
#define OLED_I2C_TIMEOUT_LOOPS                (50000U)

/*
 * Motion planner
 */

/*
 * 编码器每行驶1米产生的计数。
 *
 * 必须根据车辆实际参数填写：
 * counts/m = 每圈编码器计数 / 车轮周长(m)
 *
 * 下面的7800只是初始参考值，后续应按实测距离校准。
 * 实测7000
 */
#define CAR_ENCODER_COUNTS_PER_METER    (7000L)

/* 运动规划器更新周期。 */
#define MOTION_PLANNER_PERIOD_MS        (20U)

/* Requirement 2 keeps the uploaded acceleration/deceleration profile. */
#define MOTION_REQ2_ACCEL_MM_S2          (500L)
#define MOTION_REQ2_DECEL_MM_S2          (800L)

/*
 * Requirements 4/5/6 use a gentler launch.  At 1500 CPS and the current
 * 7000 count/m calibration, 220 mm/s^2 reaches cruise in about 0.97 s and
 * the theoretical A-to-B travel time remains below 8 s.
 */
#define MOTION_BALANCE_ACCEL_MM_S2       (220L)
#define MOTION_BALANCE_DECEL_MM_S2       (300L)

/*
 * Requirements 4/5/6: after the start marker is confirmed, keep the wheels
 * stopped briefly while the balance board receives the anticipated launch
 * acceleration and establishes a small compensating tilt.
 */
#define CAR_BALANCE_READY_TIMEOUT_MS          (1500U)

/*
 * Start pre-tilt timing for requirements 4/5/6.
 * The balance board may acknowledge earlier when the screw reaches the target.
 * MIN prevents launch before the acceleration filter has converged; MAX avoids
 * losing the test if the target keeps moving slightly with visual correction.
 */
#define CAR_BALANCE_PRETILT_MIN_MS             (220U)
#define CAR_BALANCE_PRETILT_MAX_MS             (450U)

/* UART1运动状态发送周期。 */
#define CAR_MOTION_LINK_PERIOD_MS       (20U)
#endif /* CONFIG_CAR_CONFIG_H_ */
