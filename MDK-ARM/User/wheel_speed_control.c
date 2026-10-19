#include "wheel_speed_control.h"
#include "motor.h"
#include "move.h"
#include "wheel_speed.h"

/*
 * 初始PI参数，四轮共用参数、各自保存积分状态。
 * 速度单位为0.1cm/s，控制周期为10ms。
 * P输出 = KP_X10 * error / 10。
 * I输出 = KI_X1000 * error累计值 / 1000。
 */
#define WHEEL_PI_KP_X10               30L
#define WHEEL_PI_KI_X1000             40L
#define WHEEL_PI_INTEGRAL_LIMIT       25000L

/* 约45 PWM/(cm/s)，来源于PWM500附近的实测速度。 */
#define WHEEL_FF_GAIN_X10             45L
#define WHEEL_CONTROL_PWM_LIMIT       1500L

static volatile uint8_t wheel_control_running = 0U;
static volatile int32_t wheel_target_speed_x10 = 0L;
static int32_t wheel_integral[ENCODER_NUM] = {0L, 0L, 0L, 0L};

volatile int16_t g_wheel_control_pwm[ENCODER_NUM] = {0, 0, 0, 0};

static int32_t WheelSpeedControl_Limit(int32_t value,
                                      int32_t minimum,
                                      int32_t maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void WheelSpeedControl_ClearState(void)
{
    uint8_t i;

    for (i = 0U; i < ENCODER_NUM; i++) {
        wheel_integral[i] = 0L;
        g_wheel_control_pwm[i] = 0;
    }
}

void WheelSpeedControl_Init(void)
{
    wheel_control_running = 0U;
    wheel_target_speed_x10 = 0L;
    WheelSpeedControl_ClearState();
    Move_Stop();
}

void WheelSpeedControl_Start(int32_t target_speed_x10)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    WheelSpeedControl_ClearState();
    wheel_target_speed_x10 = target_speed_x10;
    wheel_control_running = 1U;
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

void WheelSpeedControl_Stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    wheel_control_running = 0U;
    wheel_target_speed_x10 = 0L;
    WheelSpeedControl_ClearState();
    Move_Stop();
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

uint8_t WheelSpeedControl_IsRunning(void)
{
    return wheel_control_running;
}

void WheelSpeedControl_Update10ms(void)
{
    int32_t pwm[ENCODER_NUM];
    int32_t feedforward;
    uint8_t i;

    if (wheel_control_running == 0U) {
        return;
    }

    feedforward = (wheel_target_speed_x10 * WHEEL_FF_GAIN_X10) / 10L;

    for (i = 0U; i < ENCODER_NUM; i++) {
        int32_t error = wheel_target_speed_x10 - g_wheel_speed_x10[i];
        int32_t output;

        wheel_integral[i] = WheelSpeedControl_Limit(
            wheel_integral[i] + error,
            -WHEEL_PI_INTEGRAL_LIMIT,
            WHEEL_PI_INTEGRAL_LIMIT);

        output = feedforward
               + (WHEEL_PI_KP_X10 * error) / 10L
               + (WHEEL_PI_KI_X1000 * wheel_integral[i]) / 1000L;
        pwm[i] = WheelSpeedControl_Limit(output, 0L,
                                        WHEEL_CONTROL_PWM_LIMIT);
        g_wheel_control_pwm[i] = (int16_t)pwm[i];
    }

    Move_SetWheelSpeeds((int16_t)pwm[ENCODER_LF],
                        (int16_t)pwm[ENCODER_RF],
                        (int16_t)pwm[ENCODER_LR],
                        (int16_t)pwm[ENCODER_RR]);
}
