/**
 * @file    wheel_speed_control.c
 * @brief   四轮速度闭环：PI + PWM 前馈，四轮各自带符号目标
 *
 * ★本次修改★
 *   1) 目标速度从"整车一个值"改成"四轮各一个值且带符号"，
 *      后退 / 横移 / 原地旋转才有意义（原来输出限幅是 0~1500，只能正转）；
 *   2) 前馈与 PI 输出都带符号，限幅改成 -1500 ~ +1500；
 *   3) 加入积分抗饱和：输出饱和且误差同向时回退本级积分；
 *   4) 直线偏置按目标方向施加（前进时 RR 快 0.3 cm/s，后退时反向），
 *      倒车不会把原来修好的直线补偿变成"加倍跑偏"。
 */
#include "wheel_speed_control.h"
#include "motor.h"
#include "move.h"
#include "wheel_speed.h"

/*
 * PI 参数，四轮共用参数、各自保存积分状态。
 * 速度单位为 0.1cm/s，控制周期 10ms。
 *   P 输出 = KP_X10 * error / 10
 *   I 输出 = KI_X1000 * error累计值 / 1000
 */
#define WHEEL_PI_KP_X10               30L
#define WHEEL_PI_KI_X1000             40L
#define WHEEL_PI_INTEGRAL_LIMIT       25000L

/* 约 45 PWM/(cm/s)，来源于 PWM500 附近的实测速度 */
#define WHEEL_FF_GAIN_X10             45L
#define WHEEL_CONTROL_PWM_LIMIT       1500L

/* 目标速度限幅（0.1cm/s）：25 cm/s 已覆盖小车实际需要 */
#define WHEEL_TARGET_X10_LIMIT        250L

/*
 * 各轮目标速度偏置，单位 0.1 cm/s，顺序 LF、RF、LR、RR。
 * PPC 只表示编码器距离比例；直线纠偏应该在这里调目标速度。
 * 当前让 RR 比公共目标快 0.3 cm/s，用于修正车辆向右偏。
 * 符号在 WheelSpeedControl_ApplyBias() 里按目标方向给出。
 */
static const int32_t wheel_target_bias_x10[ENCODER_NUM] = {
    0L,
    0L,
    0L,
    3L
};

static volatile uint8_t wheel_control_running = 0U;
static volatile int32_t wheel_target_main_x10 = 0L;
static int32_t wheel_integral[ENCODER_NUM] = {0L, 0L, 0L, 0L};

volatile int16_t g_wheel_control_pwm[ENCODER_NUM] = {0, 0, 0, 0};
volatile int32_t g_wheel_target_x10[ENCODER_NUM] = {0L, 0L, 0L, 0L};

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

/* 偏置跟随目标方向：正转加正偏置，反转加负偏置，避免倒车跑偏加倍 */
static int32_t WheelSpeedControl_ApplyBias(uint8_t index, int32_t target_x10)
{
    int32_t bias = wheel_target_bias_x10[index];

    if (bias == 0L) {
        return 0L;
    }
    if (target_x10 > 0L) {
        return bias;
    }
    if (target_x10 < 0L) {
        return -bias;
    }
    return 0L;
}

/* 参数顺序固定为 LF、RF、LR、RR，内部换算成 Encoder_ID 下标 */
static void WheelSpeedControl_WriteTargets(int32_t lf_x10, int32_t rf_x10,
                                           int32_t lr_x10, int32_t rr_x10)
{
    int32_t target[ENCODER_NUM];
    int32_t sum = 0L;
    uint8_t i;

    target[ENCODER_LF] = WheelSpeedControl_Limit(
        lf_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);
    target[ENCODER_RF] = WheelSpeedControl_Limit(
        rf_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);
    target[ENCODER_LR] = WheelSpeedControl_Limit(
        lr_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);
    target[ENCODER_RR] = WheelSpeedControl_Limit(
        rr_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);

    for (i = 0U; i < ENCODER_NUM; i++) {
        g_wheel_target_x10[i] = target[i];
        sum += target[i];
    }
    wheel_target_main_x10 = sum / (int32_t)ENCODER_NUM;
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
    wheel_target_main_x10 = 0L;
    WheelSpeedControl_WriteTargets(0L, 0L, 0L, 0L);
    WheelSpeedControl_ClearState();
    Move_Stop();
}

void WheelSpeedControl_Start(int32_t target_speed_x10)
{
    WheelSpeedControl_StartVector(target_speed_x10, target_speed_x10,
                                  target_speed_x10, target_speed_x10);
}

void WheelSpeedControl_StartVector(int32_t lf_x10, int32_t rf_x10,
                                   int32_t lr_x10, int32_t rr_x10)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    WheelSpeedControl_ClearState();
    WheelSpeedControl_WriteTargets(lf_x10, rf_x10, lr_x10, rr_x10);
    wheel_control_running = 1U;
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

void WheelSpeedControl_SetTargets(int32_t lf_x10, int32_t rf_x10,
                                  int32_t lr_x10, int32_t rr_x10)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    WheelSpeedControl_WriteTargets(lf_x10, rf_x10, lr_x10, rr_x10);
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

void WheelSpeedControl_Stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    wheel_control_running = 0U;
    wheel_target_main_x10 = 0L;
    WheelSpeedControl_WriteTargets(0L, 0L, 0L, 0L);
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

int32_t WheelSpeedControl_GetTargetX10(void)
{
    return wheel_target_main_x10;
}

int32_t WheelSpeedControl_GetWheelTargetX10(Encoder_ID id)
{
    if (id >= ENCODER_NUM) {
        return 0L;
    }
    return g_wheel_target_x10[id];
}

void WheelSpeedControl_Update10ms(void)
{
    int32_t pwm[ENCODER_NUM];
    uint8_t i;

    if (wheel_control_running == 0U) {
        return;
    }

    for (i = 0U; i < ENCODER_NUM; i++) {
        int32_t target_x10 = g_wheel_target_x10[i]
                           + WheelSpeedControl_ApplyBias(i,
                                 g_wheel_target_x10[i]);
        int32_t feedforward = (target_x10 * WHEEL_FF_GAIN_X10) / 10L;
        int32_t error = target_x10 - g_wheel_speed_x10[i];
        int32_t output;

        wheel_integral[i] = WheelSpeedControl_Limit(
            wheel_integral[i] + error,
            -WHEEL_PI_INTEGRAL_LIMIT,
            WHEEL_PI_INTEGRAL_LIMIT);

        output = feedforward
               + (WHEEL_PI_KP_X10 * error) / 10L
               + (WHEEL_PI_KI_X1000 * wheel_integral[i]) / 1000L;

        /* 输出饱和且误差同向时回退本级积分，避免积分饱和带来超调 */
        if (output > WHEEL_CONTROL_PWM_LIMIT) {
            output = WHEEL_CONTROL_PWM_LIMIT;
            if (error > 0L) {
                wheel_integral[i] = WheelSpeedControl_Limit(
                    wheel_integral[i] - error,
                    -WHEEL_PI_INTEGRAL_LIMIT,
                    WHEEL_PI_INTEGRAL_LIMIT);
            }
        } else if (output < -WHEEL_CONTROL_PWM_LIMIT) {
            output = -WHEEL_CONTROL_PWM_LIMIT;
            if (error < 0L) {
                wheel_integral[i] = WheelSpeedControl_Limit(
                    wheel_integral[i] - error,
                    -WHEEL_PI_INTEGRAL_LIMIT,
                    WHEEL_PI_INTEGRAL_LIMIT);
            }
        }

        pwm[i] = output;
        g_wheel_control_pwm[i] = (int16_t)output;
    }

    Move_SetWheelSpeeds((int16_t)pwm[ENCODER_LF],
                        (int16_t)pwm[ENCODER_RF],
                        (int16_t)pwm[ENCODER_LR],
                        (int16_t)pwm[ENCODER_RR]);
}
