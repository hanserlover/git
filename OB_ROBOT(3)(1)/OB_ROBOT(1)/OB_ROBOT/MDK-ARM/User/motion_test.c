/**
 * @file    motion_test.c
 * @brief   单按键直行测试：CENTER 启动，走满 1.00m 自动减速停车
 *
 * 只做前进，目的是把"能不能走直"这件事放到台面上观察：
 *
 *   TRIP  本次已经走过的距离（四轮行程平均），目标 100.0cm
 *   DEV   左右两侧累计行程差（左-右）：正=左侧走得多（车身往右偏），
 *         直线纠偏就是把这个值往 0 拉，所以它应该收敛而不是一直涨
 *   LF/RF 前两个轮子的实测速度，看闭环有没有把两轮拉到同一个目标
 *
 * 按键：
 *   CENTER  启动 1m 直行；跑动中再按一次 = 中途停车
 *
 * 参数都在下面宏里：速度、距离都可以改；改完重新编译烧写即可。
 * 本文件只负责按键与显示，运动由 move.c 在 10ms 中断里执行，
 * 全程没有阻塞延时，OLED 也被限速到 100ms 刷一次。
 */
#include <stdio.h>
#include <stdio.h>
#include "motion_test.h"
#include "key.h"
#include "move.h"
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

static uint8_t  motion_key_last = KEY_NONE;
static uint32_t motion_display_tick = 0U;

/* ===================== 显示辅助 ===================== */

static void MotionTest_FormatX10(char *text, int32_t value_x10)
{
    uint32_t magnitude;
    char sign;

    if (value_x10 < 0L) {
        sign = '-';
        magnitude = (uint32_t)(-value_x10);
    } else {
        sign = '+';
        magnitude = (uint32_t)value_x10;
    }
    sprintf(text, "%c%lu.%lu", sign,
            (unsigned long)(magnitude / 10U),
            (unsigned long)(magnitude % 10U));
}

/* 0.01cm 单位的量（行程/偏差）格式化成 +43.2 这种一位小数 */
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

static void MotionTest_Show(void)
{
    char line[24];
    char lf[10];
    char rf[10];
    char trip[10];
    char dev[10];
    uint8_t running = Move_IsMoving();
    int32_t travel_x100 = Move_GetTravelX100();
    int32_t speed_lf = g_wheel_speed_x10[ENCODER_LF];
    int32_t speed_rf = g_wheel_speed_x10[ENCODER_RF];

    MotionTest_FormatX10(lf, speed_lf);
    MotionTest_FormatX10(rf, speed_rf);
    MotionTest_FormatCm(trip, travel_x100);
    MotionTest_FormatCm(dev, Move_GetStraightOffsetX100());

    OLED_ClearBuffer();

    if (running != 0U) {
        sprintf(line, "RUN %s cm", trip);
        OLED_ShowString(0, 0, line, OLED_TEXT_SIZE);
    } else if (travel_x100 != 0L) {
        sprintf(line, "DONE %s cm", trip);
        OLED_ShowString(0, 0, line, OLED_TEXT_SIZE);
    } else {
        OLED_ShowString(0, 0, "READY 1.00m", OLED_TEXT_SIZE);
    }

    sprintf(line, "TRIP %s/100.0", trip);
    OLED_ShowString(0, 16, line, OLED_TEXT_SIZE);

    sprintf(line, "LF:%s RF:%s", lf, rf);
    OLED_ShowString(0, 32, line, OLED_TEXT_SIZE);

    sprintf(line, "DEV:%s C=%s", dev, (running != 0U) ? "STOP" : "RUN");
    OLED_ShowString(0, 48, line, OLED_TEXT_SIZE);

    OLED_Refresh();
}

/* ===================== 按键 ===================== */

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

    if (key != KEY_CENTER) {
        return;                     /* 本测试只响应中间键 */
    }

    if (Move_IsMoving() != 0U) {
        Move_Halt();                /* 中途停车（斜坡减速，不甩尾） */
    } else {
        Move_DriveDistance(MOTION_TEST_SPEED_X10, MOTION_TEST_DISTANCE_X100);
    }
    MotionTest_Show();
}

/* ===================== 调度 ===================== */

void MotionTest_Init(void)
{
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
