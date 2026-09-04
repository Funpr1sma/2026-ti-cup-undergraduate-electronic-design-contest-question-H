/**
  ******************************************************************************
  * @file    motor.c
  * @brief   基于TIMA1硬件PWM的D36A STEP脉冲发生器
  ******************************************************************************
  * 硬件PWM保证脉冲周期稳定；定步模式在每个PWM周期中断里递减剩余步数，
  * 归零后立即关闭定时器并强制STEP为低，避免停止位置多出毛刺脉冲。
  ******************************************************************************
  */
#include "motor.h"

/* 中断和主循环共享的单轴运行状态，必须使用volatile。 */
static volatile uint32_t s_remaining_steps;
static volatile uint8_t s_busy;
static volatile uint8_t s_continuous;
static volatile uint8_t s_direction;

static volatile uint32_t s_frequency_hz;
/* 禁止PWM输出时强制STEP保持低电平，避免驱动器误识别边沿。 */
static void Motor_ForceLow(void)
{
    DL_TimerA_setCCPOutputDisabled(MOTOR_STEP_TIMER,
        DL_TIMER_CCP_DIS_OUT_LOW, DL_TIMER_CCP_DIS_OUT_LOW);
}

/* 恢复CCP通道由定时器输出控制。 */
static void Motor_UsePWM(void)
{
    DL_TimerA_setCCPOutputDisabled(MOTOR_STEP_TIMER,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

/*
 * 停止底层动作。调用者需要保证原子性；中断内部可直接调用，
 * 主循环调用时由外层临界区防止和计步中断同时修改状态。
 */
static void Motor_StopUnsafe(void)
{
    DL_TimerA_disableInterrupt(MOTOR_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_stopCounter(MOTOR_STEP_TIMER);
    DL_TimerA_setCaptureCompareValue(MOTOR_STEP_TIMER, 0U, MOTOR_STEP_CC_INDEX);
    Motor_ForceLow();
    s_remaining_steps = 0U;
    s_busy = 0U;
    s_continuous = 0U;
    s_frequency_hz = 0U;
}

/* 将STEP频率换算为1MHz定时器周期，并限制在16位定时器有效范围。 */
static uint16_t Motor_FrequencyToPeriod(uint32_t frequency_hz)
{
    uint32_t period = (MOTOR_STEP_TIMER_CLK_HZ + frequency_hz / 2U) / frequency_hz;
    if (period < 100U) period = 100U;
    if (period > 65535U) period = 65535U;
    return (uint16_t)period;
}

/* 配置50%占空比STEP波形；计数器从LOAD重新开始，保证首个周期完整。 */
static void Motor_ConfigurePWM(uint32_t frequency_hz)
{
    uint32_t period = Motor_FrequencyToPeriod(frequency_hz);
    uint32_t load = period - 1U;
    DL_TimerA_stopCounter(MOTOR_STEP_TIMER);
    Motor_UsePWM();
    DL_TimerA_setLoadValue(MOTOR_STEP_TIMER, load);
    DL_TimerA_setTimerCount(MOTOR_STEP_TIMER, load);
    DL_TimerA_setCaptureCompareValue(MOTOR_STEP_TIMER, period / 2U,
                                     MOTOR_STEP_CC_INDEX);
}

/* 上电默认：EN置高使能驱动器，DIR置低，STEP停止且保持低电平。 */
void Motor_Init(void)
{
    DL_GPIO_initDigitalOutputFeatures(
        IOMUX_PINCM34,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_enableOutput(
        MOTOR_EN_PORT,
        MOTOR_EN_PIN);
    DL_GPIO_setPins(
        MOTOR_EN_PORT,
        MOTOR_EN_PIN);
    DL_GPIO_clearPins(
        MOTOR_DIR_PORT,
        MOTOR_DIR_PIN);
    s_direction = 0U;
    s_frequency_hz = 0U;
    Motor_StopUnsafe();
    NVIC_ClearPendingIRQ(
        PWM_0_INST_INT_IRQN);
    NVIC_EnableIRQ(
        PWM_0_INST_INT_IRQN);
}

/* 只允许停机时改变DIR，避免运动过程中突然反向造成丢步。 */
MotorStatus_t Motor_SetDirection(MotorAxis_t axis, uint8_t high_level)
{
    if (axis != MOTOR_AXIS_X || high_level > 1U) return MOTOR_ERROR;
    if (s_busy != 0U) return MOTOR_BUSY;
    if (high_level != 0U) DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    else DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    s_direction = high_level;
    return MOTOR_OK;
}

uint8_t Motor_GetDirection(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X) ? s_direction : 0U;
}

/* 启动有限脉冲段；关中断建立临界区，保证busy和剩余步数同步更新。 */
MotorStatus_t Motor_Start(MotorAxis_t axis, uint32_t steps,
                          uint32_t frequency_hz)
{
    uint32_t primask;
    if (axis != MOTOR_AXIS_X || frequency_hz < MOTOR_MIN_FREQ_HZ ||
        frequency_hz > MOTOR_MAX_FREQ_HZ) return MOTOR_ERROR;
    if (steps == 0U) { Motor_Stop(axis); return MOTOR_OK; }
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_busy != 0U) {
        if (primask == 0U) __enable_irq();
        return MOTOR_BUSY;
    }
    Motor_ConfigurePWM(frequency_hz);
    s_remaining_steps = steps;
    s_continuous = 0U;
    s_busy = 1U;
    DL_TimerA_clearInterruptStatus(MOTOR_STEP_TIMER,
                                   DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(MOTOR_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(MOTOR_STEP_TIMER);
    if (primask == 0U) __enable_irq();
    return MOTOR_OK;
}

/* 实验2连续旋转：关闭ZERO计步中断，让硬件PWM持续自由运行。 */
void Motor_StartContinuous(uint32_t frequency_hz, uint8_t high_level)
{
    uint32_t primask;
    if (frequency_hz < MOTOR_MIN_FREQ_HZ || frequency_hz > MOTOR_MAX_FREQ_HZ) return;
    Motor_Stop(MOTOR_AXIS_X);
    (void)Motor_SetDirection(MOTOR_AXIS_X, high_level);
    primask = __get_PRIMASK();
    __disable_irq();
    Motor_ConfigurePWM(frequency_hz);
    s_remaining_steps = UINT32_MAX;
    s_continuous = 1U;
    s_busy = 1U;
    DL_TimerA_disableInterrupt(MOTOR_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(MOTOR_STEP_TIMER);
    if (primask == 0U) __enable_irq();
}
MotorStatus_t Motor_RunContinuous(
    MotorAxis_t axis,
    uint32_t frequency_hz,
    uint8_t high_level)
{
    uint32_t primask;
    if (axis != MOTOR_AXIS_X ||
        high_level > 1U ||
        frequency_hz < MOTOR_MIN_FREQ_HZ ||
        frequency_hz > MOTOR_MAX_FREQ_HZ) {
        return MOTOR_ERROR;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    /*
     * Do not overwrite a finite-step command.
     */
    if (s_busy != 0U && s_continuous == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return MOTOR_BUSY;
    }
    /*
     * The requested continuous command is already active.
     */
    if (s_busy != 0U &&
        s_continuous != 0U &&
        s_direction == high_level &&
        s_frequency_hz == frequency_hz) {
        if (primask == 0U) {
            __enable_irq();
        }
        return MOTOR_OK;
    }
    /*
     * Stop the timer before changing DIR or PWM period.
     */
    Motor_StopUnsafe();
    if (high_level != 0U) {
        DL_GPIO_setPins(
            MOTOR_DIR_PORT,
            MOTOR_DIR_PIN);
    } else {
        DL_GPIO_clearPins(
            MOTOR_DIR_PORT,
            MOTOR_DIR_PIN);
    }
    s_direction = high_level;
    Motor_ConfigurePWM(frequency_hz);
    s_remaining_steps = UINT32_MAX;
    s_continuous = 1U;
    s_busy = 1U;
    s_frequency_hz = frequency_hz;
    DL_TimerA_clearInterruptStatus(
        MOTOR_STEP_TIMER,
        DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_disableInterrupt(
        MOTOR_STEP_TIMER,
        DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(MOTOR_STEP_TIMER);
    if (primask == 0U) {
        __enable_irq();
    }
    return MOTOR_OK;
}
/* 可从主循环安全调用的停止接口。 */
void Motor_Stop(MotorAxis_t axis)
{
    uint32_t primask;
    if (axis != MOTOR_AXIS_X) return;
    primask = __get_PRIMASK();
    __disable_irq();
    Motor_StopUnsafe();
    if (primask == 0U) __enable_irq();
}

void Motor_StopAll(void) { Motor_Stop(MOTOR_AXIS_X); }
uint8_t Motor_IsBusy(MotorAxis_t axis) { return (axis == MOTOR_AXIS_X) ? s_busy : 0U; }
uint32_t Motor_GetRemainingSteps(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X) ? s_remaining_steps : 0U;
}

/**
  * STEP周期计数中断：一个ZERO事件对应完成一个完整STEP周期。
  * 连续模式不进入本计数逻辑；有限步数减到0后立即停止输出。
  */
void TIMA1_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(MOTOR_STEP_TIMER) != DL_TIMERA_IIDX_ZERO) return;
    if (s_busy == 0U || s_continuous != 0U) return;
    if (s_remaining_steps > 0U) s_remaining_steps--;
    if (s_remaining_steps == 0U) Motor_StopUnsafe();
}
