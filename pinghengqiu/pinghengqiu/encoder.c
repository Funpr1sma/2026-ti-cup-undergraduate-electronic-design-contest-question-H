/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   TIMG8硬件QEI和TIMG12 PWM组合捕获驱动
  ******************************************************************************
  * QEI计数器为16位，本模块每5ms读取一次并以int16_t差值扩展成int32_t多圈计数；
  * PWM中断记录相邻上升沿周期和上升到下降的高电平时间，再按MT6816公式换算角度。
  ******************************************************************************
  */
#include "encoder.h"
#include "demo_config.h"
#include "ti_msp_dl_config.h"

/* 优先使用SysConfig生成的实例名；条件分支仅作为缺少生成宏时的兼容后备。 */
#ifdef QEI_0_INST
#define ENCODER_QEI_INST QEI_0_INST
#else
#define ENCODER_QEI_INST TIMG8
#endif

#ifdef ENCODER_PWM_INST
#define ENCODER_PWM_TIMER      ENCODER_PWM_INST
#define ENCODER_PWM_LOAD       ENCODER_PWM_INST_LOAD_VALUE
#define ENCODER_PWM_IRQN       ENCODER_PWM_INST_INT_IRQN
#else
#define ENCODER_PWM_TIMER      TIMG12
#define ENCODER_PWM_LOAD       3999999U /* 50 ms at 80 MHz */
#define ENCODER_PWM_IRQN       TIMG12_INT_IRQn
#endif

/* 超过100ms没有有效PWM边沿，就判定编码器PWM信号丢失。 */
#define ENCODER_PWM_TIMEOUT_MS 100U

/* A/B增量位置状态：硬件16位计数经周期采样扩展为软件32位多圈计数。 */
typedef struct {
    volatile int32_t accumulated;
    int32_t zero;
    int32_t speed_last;
    uint16_t last_hw_count;
    volatile int32_t z_count;
    volatile int32_t count_at_z;
} EncoderState_t;

static EncoderState_t s_encoder;
/* PWM捕获状态由TIMG12中断写入、主循环读取，因此声明为volatile。 */
static volatile uint32_t s_pwm_last_rise;
static volatile uint32_t s_pwm_period;
static volatile uint32_t s_pwm_high;
static volatile uint32_t s_pwm_age_ms;
static volatile uint8_t s_pwm_have_rise;
static volatile uint8_t s_pwm_valid;

/* 后备QEI初始化。正常Keil工程由empty.syscfg生成QEI_0配置，本函数体会被裁掉。 */
static void Encoder_InitQEIWithoutGeneratedConfig(void)
{
#ifndef QEI_0_INST
    static DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };

    DL_TimerG_enablePower(ENCODER_QEI_INST);
    DL_TimerG_reset(ENCODER_QEI_INST);
    DL_Common_delayCycles(POWER_STARTUP_DELAY);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM2,
        IOMUX_PINCM2_PF_TIMG8_CCP0); /* PA1 = encoder A */
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM1,
        IOMUX_PINCM1_PF_TIMG8_CCP1); /* PA0 = encoder B */
    DL_TimerG_setClockConfig(ENCODER_QEI_INST, &clock_config);
    DL_TimerG_configQEI(ENCODER_QEI_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(ENCODER_QEI_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(ENCODER_QEI_INST, 0xFFFFU);
    DL_TimerG_setTimerCount(ENCODER_QEI_INST, 0U);
    DL_TimerG_enableClock(ENCODER_QEI_INST);
    DL_TimerG_startCounter(ENCODER_QEI_INST);
#endif
}

/* 后备PWM捕获初始化。正常工程由empty.syscfg生成ENCODER_PWM配置。 */
static void Encoder_InitPwmWithoutGeneratedConfig(void)
{
#ifndef ENCODER_PWM_INST
    static DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    static DL_TimerG_CaptureCombinedConfig capture_config = {
        .captureMode = DL_TIMER_CAPTURE_COMBINED_MODE_PULSE_WIDTH_AND_PERIOD,
        .period = ENCODER_PWM_LOAD,
        .startTimer = DL_TIMER_START,
        .inputChan = DL_TIMER_INPUT_CHAN_0,
        .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT
    };

    DL_TimerG_enablePower(ENCODER_PWM_TIMER);
    DL_TimerG_reset(ENCODER_PWM_TIMER);
    DL_Common_delayCycles(POWER_STARTUP_DELAY);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM48,
        IOMUX_PINCM48_PF_TIMG12_CCP0); /* PB20 = encoder PWM */
    DL_TimerG_setClockConfig(ENCODER_PWM_TIMER, &clock_config);
    DL_TimerG_initCaptureCombinedMode(ENCODER_PWM_TIMER, &capture_config);
    DL_TimerG_enableInterrupt(ENCODER_PWM_TIMER,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT | DL_TIMERG_INTERRUPT_CC1_DN_EVENT);
    DL_TimerG_enableClock(ENCODER_PWM_TIMER);
#endif
    NVIC_ClearPendingIRQ(ENCODER_PWM_IRQN);
    NVIC_EnableIRQ(ENCODER_PWM_IRQN);
}

/* TIMG向下计数，计算两个捕获时刻的间隔时同时处理计数器回绕。 */
static uint32_t Encoder_PwmElapsed(uint32_t older, uint32_t newer)
{
    if (older >= newer) return older - newer;
    return older + (ENCODER_PWM_LOAD + 1U - newer);
}

/* 读取QEI当前逻辑方向，并应用ENCODER_AXIS_X_SIGN保持对外正负方向一致。 */
static int8_t Encoder_GetDirectionSign(void)
{
    uint8_t counting_down =
        (DL_TimerG_getQEIDirection(ENCODER_QEI_INST) == DL_TIMER_QEI_DIR_DOWN) ? 1U : 0U;
#if (ENCODER_AXIS_X_SIGN < 0)
    counting_down = (uint8_t)!counting_down;
#endif
    return (counting_down != 0U) ? -1 : 1;
}

/* 在Z中断时读取更接近边沿时刻的A/B位置，不等待下一次5ms周期采样。 */
static int32_t Encoder_GetCountAtCurrentEdge(void)
{
    uint16_t now = (uint16_t)DL_TimerG_getTimerCount(ENCODER_QEI_INST);
    int16_t delta = (int16_t)(now - s_encoder.last_hw_count);
#if (ENCODER_AXIS_X_SIGN < 0)
    delta = (int16_t)-delta;
#endif
    return s_encoder.accumulated + (int32_t)delta - s_encoder.zero;
}
/* 清空软件状态，上电当前位置作为A/B累计计数的起点。 */
void Encoder_Init(void)
{
    s_pwm_last_rise = 0U;
    s_pwm_period = 0U;
    s_pwm_high = 0U;
    s_pwm_age_ms = ENCODER_PWM_TIMEOUT_MS + 1U;
    s_pwm_have_rise = 0U;
    s_pwm_valid = 0U;
    Encoder_InitQEIWithoutGeneratedConfig();
    Encoder_InitPwmWithoutGeneratedConfig();
    DL_TimerG_setTimerCount(ENCODER_QEI_INST, 0U);
    DL_TimerG_startCounter(ENCODER_QEI_INST);
    s_encoder.accumulated = 0;
    s_encoder.zero = 0;
    s_encoder.speed_last = 0;
    s_encoder.last_hw_count = 0U;
    s_encoder.z_count = 0;
    s_encoder.count_at_z = 0;
    NVIC_ClearPendingIRQ(ENCODER_Z_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_Z_INT_IRQN);
}

/**
  * 5ms周期采样QEI。
  * 把16位无符号差值强制转换为int16_t，可在单周期位移小于32768计数时
  * 自动处理0和65535之间的正反向回绕，再累加成32位多圈位置。
  */
void Encoder_Tick(uint32_t elapsed_ms)
{
    uint16_t now;
    int16_t delta;
    now = (uint16_t)DL_TimerG_getTimerCount(ENCODER_QEI_INST);
    delta = (int16_t)(now - s_encoder.last_hw_count);
    s_encoder.last_hw_count = now;
#if (ENCODER_AXIS_X_SIGN < 0)
    delta = (int16_t)-delta;
#endif
    s_encoder.accumulated += (int32_t)delta;
    if (s_pwm_age_ms <= ENCODER_PWM_TIMEOUT_MS) s_pwm_age_ms += elapsed_ms;
    if (s_pwm_age_ms > ENCODER_PWM_TIMEOUT_MS) s_pwm_valid = 0U;
}

int32_t Encoder_GetCount(EncoderAxis_t axis)
{
    if (axis != ENCODER_AXIS_X) return 0;
    return s_encoder.accumulated - s_encoder.zero;
}

float Encoder_GetAngle(EncoderAxis_t axis)
{
    return (float)Encoder_GetCount(axis) * 360.0f / ENCODER_COUNTS_PER_REV;
}

int32_t Encoder_GetRawCount(EncoderAxis_t axis)
{
    if (axis != ENCODER_AXIS_X) return 0;
    return s_encoder.accumulated;
}

float Encoder_GetRawAngle(EncoderAxis_t axis)
{
    return (float)Encoder_GetRawCount(axis) * 360.0f /
           ENCODER_COUNTS_PER_REV;
}

/* 速度=计数增量/每圈计数*360度/采样时间。 */
float Encoder_CalcSpeedDps(EncoderAxis_t axis, uint32_t period_ms)
{
    int32_t now, delta;
    if (axis != ENCODER_AXIS_X || period_ms == 0U) return 0.0f;
    now = Encoder_GetCount(axis);
    delta = now - s_encoder.speed_last;
    s_encoder.speed_last = now;
    return (float)delta * 360000.0f /
           ((float)ENCODER_COUNTS_PER_REV * (float)period_ms);
}

/* 软件清零只修改零点偏移，不强制改写硬件QEI计数器。 */
void Encoder_SetZero(EncoderAxis_t axis)
{
    if (axis != ENCODER_AXIS_X) return;
    s_encoder.zero = s_encoder.accumulated;
    s_encoder.speed_last = 0;
    s_encoder.z_count = 0;
    s_encoder.count_at_z = 0;
}

/* 软件清零只修改零点偏移，不强制改写硬件QEI计数器。 */
void Encoder_SetZeroAll(void) { Encoder_SetZero(ENCODER_AXIS_X); }

/* 读取物理Z上升沿累计的有符号净圈数。 */
int32_t Encoder_GetZCount(EncoderAxis_t axis)
{
    return (axis == ENCODER_AXIS_X) ? s_encoder.z_count : 0;
}

int32_t Encoder_GetCountAtLastZ(EncoderAxis_t axis)
{
    return (axis == ENCODER_AXIS_X) ? s_encoder.count_at_z : 0;
}

/**
  * PWM占空比转绝对角度。
  * 参考MS42CG/MT6816公式：位置码=duty*4115-1，再映射到0~360度。
  */
uint8_t Encoder_GetPwmAngle(EncoderAxis_t axis, float *angle_deg)
{
    float duty;
    float angle;
    uint32_t period;
    uint32_t high;
    if (axis != ENCODER_AXIS_X || angle_deg == 0 || s_pwm_valid == 0U) return 0U;
    period = s_pwm_period;
    high = s_pwm_high;
    if (period == 0U || high == 0U || high >= period) return 0U;
    duty = (float)high / (float)period;
    angle = ((duty * 4115.0f) - 1.0f) * 360.0f / 4115.0f;
    if (angle < 0.0f) angle = 0.0f;
    if (angle >= 360.0f) angle -= 360.0f;
    *angle_deg = angle;
    return 1U;
}

/**
  * PWM输入捕获中断：CC1记录上升沿并计算周期，CC0记录下降沿并计算高电平。
  * 只有0<高电平<周期时才发布有效数据并刷新超时计时器。
  */
void TIMG12_IRQHandler(void)
{
    DL_TIMER_IIDX pending;
    do {
        pending = DL_TimerG_getPendingInterrupt(ENCODER_PWM_TIMER);
        if (pending == DL_TIMERG_IIDX_CC1_DN) {
            uint32_t rise = DL_TimerG_getCaptureCompareValue(
                ENCODER_PWM_TIMER, DL_TIMER_CC_1_INDEX);
            if (s_pwm_have_rise != 0U) {
                s_pwm_period = Encoder_PwmElapsed(s_pwm_last_rise, rise);
            }
            s_pwm_last_rise = rise;
            s_pwm_have_rise = 1U;
        } else if (pending == DL_TIMERG_IIDX_CC0_DN) {
            uint32_t fall = DL_TimerG_getCaptureCompareValue(
                ENCODER_PWM_TIMER, DL_TIMER_CC_0_INDEX);
            if (s_pwm_have_rise != 0U) {
                uint32_t high = Encoder_PwmElapsed(s_pwm_last_rise, fall);
                s_pwm_high = high;
                if (s_pwm_period > high && high != 0U) {
                    s_pwm_age_ms = 0U;
                    s_pwm_valid = 1U;
                }
            }
        }
    } while ((uint32_t)pending != 0U);
}
/**
  * 编码器Z上升沿中断。
  * Z信号本身不带方向，必须同时读取硬件QEI方向，才能得到有符号净圈数。
  */
void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(ENCODER_Z_PORT)) {
        case ENCODER_Z_Z_IIDX:
            s_encoder.z_count += Encoder_GetDirectionSign();
            s_encoder.count_at_z = Encoder_GetCountAtCurrentEdge();
            break;
        default:
            break;
    }
}