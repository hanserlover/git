/**
 * @file    motion_test.c
 * @brief   四方向直行测试：方向键各走 1 米，全程保持航向（角度环）
 *
 * 按键：
 *   UP     前进 1m        DOWN   后退 1m
 *   LEFT   左移 1m        RIGHT  右移 1m
 *   CENTER 停止（斜坡减速）
 *
 * 屏幕上四个量：
 *   TRIP  本次沿指令方向走过的距离（里程计投影），目标 100.0cm
 *   YAW   当前航向（0.1°，逆时针为正）
 *   E     角度环的航向误差（0.1°），应被压到 ±0.5° 以内
 *   ODO / IMU   航向来源：里程计 / 外部陀螺
 *
 * 四个方向走的是同一套解算和同一个角度环，所以"能走直"这件事在四个方向
 * 上应该表现一致：起转时 YAW 会动一下，随后被拉回 0 附近。
 *
 * 本文件只做按键与显示；控制全部在 10ms 中断里，没有阻塞延时。
 */
#include <stdio.h>
#include "motion_test.h"
#include "key.h"
#include "move.h"
#include "heading.h"
#include "oled.h"
#include "wheel_speed.h"

/* ===================== 可调参数 ===================== */

/* 测试速度（0.1 cm/s）：100 = 10 cm/s */
#define MOTION_TEST_SPEED_X10         100L

/* 目标距离（0.01 cm）：10000 = 1.00 m */
#define MOTION_TEST_DISTANCE_X100     10000L

/* OLED 刷新周期，单位 10ms（10 = 100ms） */
#define MOTION_TEST_DISPLAY_TICKS     10U

#define OLED_TEXT_SIZE                12U

/* ===================== 内部状态 ===================== */

typedef enum {
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_FORWARD,
    MOTION_MODE_BACKWARD,
    MOTION_MODE_LEFT,
    MOTION_MODE_RIGHT
} MotionTest_Mode;

static MotionTest_Mode motion_mode = MOTION_MODE_IDLE;
static uint8_t  motion_key_last = KEY_NONE;
static uint32_t motion_display_tick = 0U;

/* ===================== 显示辅助 ===================== */

/* 一位小数：0.1cm/s 的速度、0.1° 的角度都用它 */
static void MotionTest_FormatX10(char *text, int32_t value)
{
    uint32_t magnitude;
    char sign;

    if (value < 0L) {
        sign = '-';
        magnitude = (uint32_t)(-value);
    } else {
        sign = '+';
        magnitude = (uint32_t)value;
    }
    sprintf(text, "%c%lu.%lu", sign,
            (unsigned long)(magnitude / 10U),
            (unsigned long)(magnitude % 10U));
}

/* 0.01cm 单位的量（行程）格式化成 +43.2 */
static void MotionTest_FormatCm(char *text, int32_t value_x100)
{
    uint32_t magnitude;
    char sign;

    if (value_x100 < 0L) {
        sign = '-';
        magnitude = (uint32_t)(-value_x100);
    } else {
        sign = '+';
        magnitude = (uint32_t)value_x100;
    }
    sprintf(text, "%c%lu.%lu", sign,
            (unsigned long)(magnitude / 100U),
            (unsigned long)((magnitude % 100U) / 10U));
}

static const char *MotionTest_ModeName(MotionTest_Mode mode)
{
    switch (mode) {
        case MOTION_MODE_FORWARD:   return "FWD";
        case MOTION_MODE_BACKWARD:  return "BACK";
        case MOTION_MODE_LEFT:      return "LEFT";
        case MOTION_MODE_RIGHT:     return "RIGHT";
        case MOTION_MODE_IDLE:
        default:                    return "IDLE";
    }
}

static void MotionTest_Show(void)
{
    char line[24];
    char lf[10];
    char rf[10];
    char trip[10];
    char yaw[10];
    char err[10];
    uint8_t running = Move_IsMoving();
    int32_t travel_x100 = Move_GetTravelX100();

    MotionTest_FormatX10(lf, g_wheel_speed_x10[ENCODER_LF]);
    MotionTest_FormatX10(rf, g_wheel_speed_x10[ENCODER_RF]);
    MotionTest_FormatCm(trip, travel_x100);
    MotionTest_FormatX10(yaw, Heading_GetYawX10());
    MotionTest_FormatX10(err, Heading_GetErrorX10());

    OLED_ClearBuffer();

    /* 第 1 行：方向 + 状态 + 提示 */
    if (running != 0U) {
        sprintf(line, "%-5s RUN  C=STOP", MotionTest_ModeName(motion_mode));
        OLED_ShowString(0, 0, line, OLED_TEXT_SIZE);
    } else if (travel_x100 != 0L) {
        sprintf(line, "%-5s DONE", MotionTest_ModeName(motion_mode));
        OLED_ShowString(0, 0, line, OLED_TEXT_SIZE);
    } else {
        OLED_ShowString(0, 0, "READY: UP DN L R", OLED_TEXT_SIZE);
    }

    /* 第 2 行：本次行程 / 目标 */
    sprintf(line, "TRIP %s/100.0", trip);
    OLED_ShowString(0, 16, line, OLED_TEXT_SIZE);

    /* 第 3 行：航向 + 航向误差 + 航向来源 */
    sprintf(line, "YAW%s E%s %s", yaw, err,
            (Heading_GetSource() != 0U) ? "IMU" : "ODO");
    OLED_ShowString(0, 32, line, OLED_TEXT_SIZE);

    /* 第 4 行：跑动中看轮速，停下来看按键说明 */
    if (running != 0U) {
        sprintf(line, "LF:%s RF:%s", lf, rf);
        OLED_ShowString(0, 48, line, OLED_TEXT_SIZE);
    } else {
        OLED_ShowString(0, 48, "C=STOP U/D/L/R GO", OLED_TEXT_SIZE);
    }

    OLED_Refresh();
}

/* ===================== 按键 ===================== */

static void MotionTest_Start(MotionTest_Mode mode, int32_t vx_x10, int32_t vy_x10)
{
    motion_mode = mode;
    Move_DriveDistance(vx_x10, vy_x10, MOTION_TEST_DISTANCE_X100);
    MotionTest_Show();
}

void MotionTest_ProcessKey(uint8_t key)
{
    if (key == KEY_NONE) {
        motion_key_last = KEY_NONE;
        return;
    }
    if (key == motion_key_last) {
        return;                     /* 只处理按下那一下 */
    }
    motion_key_last = key;

    switch (key) {
        case KEY_UP:
            MotionTest_Start(MOTION_MODE_FORWARD, MOTION_TEST_SPEED_X10, 0L);
            break;
        case KEY_DOWN:
            MotionTest_Start(MOTION_MODE_BACKWARD, -MOTION_TEST_SPEED_X10, 0L);
            break;
        case KEY_LEFT:
            MotionTest_Start(MOTION_MODE_LEFT, 0L, MOTION_TEST_SPEED_X10);
            break;
        case KEY_RIGHT:
            MotionTest_Start(MOTION_MODE_RIGHT, 0L, -MOTION_TEST_SPEED_X10);
            break;
        case KEY_CENTER:
            Move_Halt();
            MotionTest_Show();
            break;
        default:
            break;
    }
}

/* ===================== 调度 ===================== */

void MotionTest_Init(void)
{
    motion_mode = MOTION_MODE_IDLE;
    motion_key_last = KEY_NONE;
    motion_display_tick = g_tick_10ms;

    Move_Halt();
    MotionTest_Show();
}

void MotionTest_ProcessTick(uint32_t tick_10ms)
{
    if ((tick_10ms - motion_display_tick) >= MOTION_TEST_DISPLAY_TICKS) {
        motion_display_tick = tick_10ms;
        MotionTest_Show();
    }
}
