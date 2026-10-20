/**
 * @file    heading.c
 * @brief   航向角环（外环）+ 编码器里程计实现
 *
 * 详见 heading.h 的说明。这里只强调三件事：
 *   1) 里程计用的是"位移"而不是"速度"，所以走多远、转多少都不会漂出量纲；
 *   2) 角度环是 PI：纯 P 在"两侧轮子有效半径不一致"这类恒定扰动下会留下
 *      固定航向偏差，加 I 才能把稳态误差压到 0；
 *   3) 只在直行指令期间生效（运动层 Heading_Hold/Release 控制），
 *      旋转指令期间绝不干预，否则会跟人的意图打架。
 */
#include "heading.h"
#include "encoder.h"
#include "wheel_speed.h"

/* ##########################################################################
 * #                          ★ 参 数 区 ★                                  #
 * ########################################################################## */

/* ------------------------------------------------------------------------
 * 左右轮距（左右两侧轮子中心线之间的距离），单位 0.1cm：160 = 16.0cm
 * ------------------------------------------------------------------------
 * 这个值只影响"轮子位移差 -> 车身转角"的换算比例。
 * 量法：两个左侧轮子中心到两个右侧轮子中心的距离（不是车宽）。
 * 偏大 → 角度环以为转得少，修正偏弱；偏小 → 修正偏猛、容易来回摆。
 */
#define HEADING_TRACK_X10             160L

/* dφ(0.1°) = d_rot(0.01cm) * HEADING_YAW_NUM / HEADING_YAW_DEN
 * HEADING_YAW_DEN = round(π * 轮距_0.1cm) */
#define HEADING_YAW_NUM               360L
#define HEADING_YAW_DEN               503L    /* round(3.14159 * 160) */

/* ------------------------------------------------------------------------
 * 角度环 PI（误差单位 0.1°，输出单位 0.1cm/s）
 * ------------------------------------------------------------------------
 * KP：每 1° 误差给 6 个单位的旋转修正（5° -> 3cm/s）
 * KI：每 1° 误差持续 1s 累积 3 个单位；积分时间 Ti ≈ Kp/Ki ≈ 2s，
 *     比角度环本身的响应（≈0.23s）慢一个数量级，避免积分引起摆动
 * 死区：0.3° 以内不积分（P 项照常出力），避免里程计量化噪声喂给积分
 * 输出上限：6cm/s，打滑时不至于猛打方向
 */
#define HEADING_KP                    60L
#define HEADING_KP_DIV                100L
#define HEADING_KI                    30L
#define HEADING_KI_DIV                10000L
#define HEADING_DEADBAND_01DEG        3L
#define HEADING_WZ_LIMIT_X10          60L

/* 外部航向源超时：连续这么多周期（10ms）没有新样例会退回里程计 */
#define HEADING_EXT_TIMEOUT_TICKS     20U

/* ##########################################################################
 * #                          参数区结束                                     #
 * ########################################################################## */

volatile int32_t g_odom_vx_x100 = 0L;
volatile int32_t g_odom_vy_x100 = 0L;

static int32_t heading_last_dist[ENCODER_NUM] = {0L, 0L, 0L, 0L};
static int32_t heading_yaw_01deg = 0L;
static int32_t heading_target_01deg = 0L;
static int32_t heading_integral = 0L;
static int32_t heading_correction_x10 = 0L;
static uint8_t heading_holding = 0U;
static uint8_t heading_source = 0U;          /* 0=里程计 1=外部 */
static uint8_t heading_ext_age = 0U;

static int32_t Heading_Limit(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

/* 航向归一化到 ±180.0°（±1800） */
static int32_t Heading_Normalize(int32_t angle_01deg)
{
    while (angle_01deg > 1800L) {
        angle_01deg -= 3600L;
    }
    while (angle_01deg < -1800L) {
        angle_01deg += 3600L;
    }
    return angle_01deg;
}

/* 角度环本体：航向误差 -> wz 修正量 */
static void Heading_Control(void)
{
    int32_t error;
    int32_t output;

    if (heading_holding == 0U) {
        heading_integral = 0L;
        heading_correction_x10 = 0L;
        return;
    }

    error = Heading_Normalize(heading_target_01deg - heading_yaw_01deg);

    /* 死区外才积分，抑制里程计量化噪声带来的积分漂移 */
    if ((error >= HEADING_DEADBAND_01DEG)
        || (error <= -HEADING_DEADBAND_01DEG)) {
        heading_integral += (error * HEADING_KI) / HEADING_KI_DIV;
        heading_integral = Heading_Limit(heading_integral,
                                         -HEADING_WZ_LIMIT_X10,
                                         HEADING_WZ_LIMIT_X10);
    }

    output = (error * HEADING_KP) / HEADING_KP_DIV + heading_integral;
    /* 航向偏左（error>0 表示车头比目标偏向顺时针）需要左旋，所以直接相加 */
    heading_correction_x10 = Heading_Limit(output,
                                           -HEADING_WZ_LIMIT_X10,
                                           HEADING_WZ_LIMIT_X10);
}

void Heading_Reset(void)
{
    uint8_t i;

    for (i = 0U; i < ENCODER_NUM; i++) {
        heading_last_dist[i] = g_wheel_distance_x100[i];
    }
    g_odom_vx_x100 = 0L;
    g_odom_vy_x100 = 0L;
    heading_yaw_01deg = 0L;
    heading_target_01deg = 0L;
    heading_integral = 0L;
    heading_correction_x10 = 0L;
}

void Heading_Init(void)
{
    heading_holding = 0U;
    heading_source = 0U;
    heading_ext_age = 0U;
    Heading_Reset();
}

void Heading_Update10ms(void)
{
    int32_t delta[ENCODER_NUM];
    int32_t d_vx;
    int32_t d_vy;
    int32_t d_rot;
    uint8_t i;

    /* 1) 取本周期四轮位移增量（0.01cm） */
    for (i = 0U; i < ENCODER_NUM; i++) {
        int32_t now = g_wheel_distance_x100[i];

        delta[i] = now - heading_last_dist[i];
        heading_last_dist[i] = now;
    }

    /* 2) 麦轮逆运动学：位移版（与速度版是同一套矩阵） */
    d_vx  = ( delta[ENCODER_LF] + delta[ENCODER_RF]
            + delta[ENCODER_LR] + delta[ENCODER_RR]) / 4;
    d_vy  = ( delta[ENCODER_LF] - delta[ENCODER_RF]
            - delta[ENCODER_LR] + delta[ENCODER_RR]) / 4;
    d_rot = (-delta[ENCODER_LF] + delta[ENCODER_RF]
             - delta[ENCODER_LR] + delta[ENCODER_RR]) / 4;

    g_odom_vx_x100 += d_vx;
    g_odom_vy_x100 += d_vy;

    /* 3) 航向：外部源超时就退回里程计积分 */
    if (heading_source != 0U) {
        if (heading_ext_age < 0xFFU) {
            heading_ext_age++;
        }
        if (heading_ext_age > HEADING_EXT_TIMEOUT_TICKS) {
            heading_source = 0U;
        }
    }
    if (heading_source == 0U) {
        heading_yaw_01deg += (d_rot * HEADING_YAW_NUM) / HEADING_YAW_DEN;
        heading_yaw_01deg = Heading_Normalize(heading_yaw_01deg);
    }

    /* 4) 角度环 */
    Heading_Control();
}

void Heading_Hold(void)
{
    heading_target_01deg = heading_yaw_01deg;
    heading_integral = 0L;
    heading_correction_x10 = 0L;
    heading_holding = 1U;
}

void Heading_Release(void)
{
    heading_holding = 0U;
    heading_integral = 0L;
    heading_correction_x10 = 0L;
}

uint8_t Heading_IsHolding(void)
{
    return heading_holding;
}

void Heading_SetTargetX10(int32_t yaw_01deg)
{
    heading_target_01deg = Heading_Normalize(yaw_01deg);
    heading_integral = 0L;
}

int32_t Heading_GetTargetX10(void)
{
    return heading_target_01deg;
}

int32_t Heading_GetYawX10(void)
{
    return heading_yaw_01deg;
}

int32_t Heading_GetErrorX10(void)
{
    return Heading_Normalize(heading_target_01deg - heading_yaw_01deg);
}

int32_t Heading_GetCorrectionX10(void)
{
    return heading_correction_x10;
}

void Heading_InputExternalYaw(int32_t yaw_01deg, uint8_t valid)
{
    if (valid == 0U) {
        heading_source = 0U;
        heading_ext_age = 0U;
        return;
    }

    if (heading_source == 0U) {
        /* 第一次切到外部源：把内部航向对齐过去，避免目标角突然跳变 */
        heading_target_01deg += (yaw_01deg - heading_yaw_01deg);
        heading_target_01deg = Heading_Normalize(heading_target_01deg);
    }
    heading_source = 1U;
    heading_ext_age = 0U;
    heading_yaw_01deg = Heading_Normalize(yaw_01deg);
}

uint8_t Heading_GetSource(void)
{
    return heading_source;
}
