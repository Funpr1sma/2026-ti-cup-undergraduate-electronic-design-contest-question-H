#ifndef DEMO_CONFIG_H_
#define DEMO_CONFIG_H_

/* MS42CG A/B: 1000-line quadrature encoder, x4 = 4000 counts/rev. */
#define ENCODER_COUNTS_PER_REV      4000U

/* Change to -1 only when positive mechanical motion makes the count decrease. */
#define ENCODER_AXIS_X_SIGN         1

/* DIR level expected to make the public encoder count increase. */
#define AXIS_X_POSITIVE_DIR_LEVEL   1U

/* Stepper position-loop scheduler and arrival tolerance. */
#define CL_PERIOD_MS                5U
#define CL_TOLERANCE_COUNTS         2U

#endif /* DEMO_CONFIG_H_ */
