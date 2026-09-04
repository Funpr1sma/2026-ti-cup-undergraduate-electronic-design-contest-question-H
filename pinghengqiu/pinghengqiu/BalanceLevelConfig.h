#ifndef BALANCE_LEVEL_CONFIG_H_
#define BALANCE_LEVEL_CONFIG_H_

/*
 * Encoder raw angle used by levelgo immediately after power-up.
 *
 * The AS5600/multi-turn raw angle in this project is referenced to the motor
 * position present at MCU reset. Therefore 0 deg is the safest default when
 * the mechanism is normally powered on near its mechanical level position.
 * The user may refine it at runtime with levelsave or leveldefault=<raw_deg>.
 */
#define BALANCE_LEVEL_DEFAULT_RAW_DEG        (0.0f)
#define BALANCE_LEVEL_DEFAULT_VALID_ON_BOOT  (1U)

#endif /* BALANCE_LEVEL_CONFIG_H_ */
