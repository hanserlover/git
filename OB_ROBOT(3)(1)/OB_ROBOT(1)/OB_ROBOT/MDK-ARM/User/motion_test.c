/**
 * @file    motion_test.c
 * @brief   四向运动测试界面：按键选方向，走速度闭环，OLED 显示四轮实测速度
 *
 * 按键：
 *   UP     前进      再按一次同方向键 = 停车
 *   DOWN   后退
 *   LEFT   左平移    长按 400ms 变成原地左转
 *   RIGHT  右平移    长按 400ms 变成原地右转
 *   CENTER 停车（按斜坡减速，不会甩尾）
 *
 * 想直接让 LEFT/RIGHT 就是原地旋转，把 MOTION_TEST_ROTATE_WITH_LR 改成 1，
 * 左右平移改用 Move_Left()/Move_Right() 在代码里调用即可。
 *
 * 注意：本文件只负责按键语义与显示，真正的运动由 move.c 在 10ms 中断里执行；
 * 这里不会调用任何阻塞延时，OLED 刷新也被限速到 100ms 一次。
 */
#include <stdio.h>
#include "motion_test.h"
#include "key.h"
#include "move.h"
#include "oled.h"
#include "wheel_speed.h"

/* ===================== 可调参数 ===================== */

/* 测试用速度（0.1 cm/s）：100 = 10 cm/s */
#define MOTION_TEST_SPEED_X10         100L

/* OLED 刷新周期，单位 10ms（10 = 100ms） */
#define MOTION_TEST_DISPLAY_TICKS     10U

/* 长按判定：40 个 10ms = 400ms */
#define MOTION_TEST_HOLD_TICKS        40U

/* 0 = LEFT/RIGHT 做左右平移，长按做原地旋转；1 = LEFT/RIGHT 直接做原地旋转 */
#define MOTION_TEST_ROTATE_WITH_LR    0

#define OLED_TEXT_SIZE                12U

/* ===================== 内部状态 ===================== */

typedef enum {
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_FORWARD,
    MOTION_MODE_BACKWARD,
    MOTION_MODE_LEFT,
    MOTION_MODE_RIGHT,
    MOTION_MODE_ROTATE_LEFT,
    MOTION_MODE_ROTATE_RIGHT
} MotionTest_Mode;

#if (MOTION_TEST_ROTATE_WITH_LR != 0)
#define MOTION_TEST_LR_LEFT_MODE    MOTION_MODE_ROTATE_LEFT
#define MOTION_TEST_LR_RIGHT_MODE   MOTION_MODE_ROTATE_RIGHT
#else
#define MOTION_TEST_LR_LEFT_MODE    MOTION_MODE_LEFT
#define MOTION_TEST_LR_RIGHT_MODE   MOTION_MODE_RIGHT
#endif

static MotionTest_Mode motion_mode = MOTION_MODE_IDLE;
static int32_t  motion_speed_x10 = MOTION_TEST_SPEED_X10;
static uint8_t  motion_key_last = KEY_NONE;
static uint8_t  motion_hold_ticks = 0U;
static uint8_t  motion_hold_fired = 0U;
static uint32_t motion_display_tick = 0U;

/* ===================== 显示辅助 ===================== */

static const char *MotionTest_ModeName(MotionTest_Mode mode)
{
    switch (mode) {
        case MOTION_MODE_FORWARD:       return "FWD";
        case MOTION_MODE_BACKWARD:      return "BACK";
        case MOTION_MODE_LEFT:          return "LEFT";
        case MOTION_MODE_RIGHT:         return "RIGHT";
        case MOTION_MODE_ROTATE_LEFT:   return "ROT-L";
        case MOTION_MODE_ROTATE_RIGHT:  return "ROT-R";
        case MOTION_MODE_IDLE:
        default:                        return "STOP";
    }
}

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

/* 0.01cm 单位的量（左右行程差）格式化成 +1.2 这种一位小数 */
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
    char lr[10];
    char rr[10];
    char dr[10];
    int32_t speed_lf = g_wheel_speed_x10[ENCODER_LF];
    int32_t speed_rf = g_wheel_speed_x10[ENCODER_RF];
    int32_t speed_lr = g_wheel_speed_x10[ENCODER_LR];
    int32_t speed_rr = g_wheel_speed_x10[ENCODER_RR];

    MotionTest_FormatX10(lf, speed_lf);
    MotionTest_FormatX10(rf, speed_rf);
    MotionTest_FormatX10(lr, speed_lr);
    MotionTest_FormatX10(rr, speed_rr);
    MotionTest_FormatCm(dr, Move_GetStraightOffsetX100());

    OLED_ClearBuffer();

    sprintf(line, "%-6s T%lu.%lu", MotionTest_ModeName(motion_mode),
            (unsigned long)(motion_speed_x10 / 10L),
            (unsigned long)(motion_speed_x10 % 10L));
    OLED_ShowString(0, 0, line, OLED_TEXT_SIZE);

    sprintf(line, "LF:%s RF:%s", lf, rf);
    OLED_ShowString(0, 16, line, OLED_TEXT_SIZE);

    sprintf(line, "LR:%s RR:%s", lr, rr);
    OLED_ShowString(0, 32, line, OLED_TEXT_SIZE);

    if (motion_mode == MOTION_MODE_IDLE) {
        OLED_ShowString(0, 48, "UP/DN F/B L/R SIDE", OLED_TEXT_SIZE);
    } else {
        /* D = 左右累计行程差（cm），直线纠偏把它往 0 拉 */
        sprintf(line, "D:%s C=STOP", dr);
        OLED_ShowString(0, 48, line, OLED_TEXT_SIZE);
    }
    OLED_Refresh();
}

/* ===================== 指令下发 ===================== */

static void MotionTest_Apply(void)
{
    switch (motion_mode) {
        case MOTION_MODE_FORWARD:
            Move_Forward(motion_speed_x10);
            break;
        case MOTION_MODE_BACKWARD:
            Move_Backward(motion_speed_x10);
            break;
        case MOTION_MODE_LEFT:
            Move_Left(motion_speed_x10);
            break;
        case MOTION_MODE_RIGHT:
            Move_Right(motion_speed_x10);
            break;
        case MOTION_MODE_ROTATE_LEFT:
            Move_RotateLeft(motion_speed_x10);
            break;
        case MOTION_MODE_ROTATE_RIGHT:
            Move_RotateRight(motion_speed_x10);
            break;
        case MOTION_MODE_IDLE:
        default:
            Move_Halt();
            break;
    }
}

static void MotionTest_SetMode(MotionTest_Mode mode)
{
    motion_mode = mode;
    MotionTest_Apply();
    MotionTest_Show();
}

/* 同方向再按一次 = 停车 */
static void MotionTest_Toggle(MotionTest_Mode mode)
{
    if (motion_mode == mode) {
        MotionTest_SetMode(MOTION_MODE_IDLE);
    } else {
        MotionTest_SetMode(mode);
    }
}

/* ===================== 按键处理 ===================== */

static void MotionTest_EdgeAction(uint8_t key)
{
    switch (key) {
        case KEY_UP:
            MotionTest_Toggle(MOTION_MODE_FORWARD);
            break;
        case KEY_DOWN:
            MotionTest_Toggle(MOTION_MODE_BACKWARD);
            break;
        case KEY_LEFT:
            MotionTest_Toggle(MOTION_TEST_LR_LEFT_MODE);
            break;
        case KEY_RIGHT:
            MotionTest_Toggle(MOTION_TEST_LR_RIGHT_MODE);
            break;
        case KEY_CENTER:
            MotionTest_SetMode(MOTION_MODE_IDLE);
            break;
        default:
            break;
    }
}

/* 长按 LEFT/RIGHT：切成原地旋转 */
static void MotionTest_HoldAction(uint8_t key)
{
    if (key == KEY_LEFT) {
        MotionTest_SetMode(MOTION_MODE_ROTATE_LEFT);
    } else if (key == KEY_RIGHT) {
        MotionTest_SetMode(MOTION_MODE_ROTATE_RIGHT);
    }
}

void MotionTest_ProcessKey(uint8_t key)
{
    if (key == KEY_NONE) {
        motion_key_last = KEY_NONE;
        motion_hold_ticks = 0U;
        motion_hold_fired = 0U;
        return;
    }

    /* 新按下的键：做一次边沿动作 */
    if (key != motion_key_last) {
        motion_key_last = key;
        motion_hold_ticks = 1U;
        motion_hold_fired = 0U;
        MotionTest_EdgeAction(key);
        return;
    }

    /* 持续按住：累计长按时间，到阈值只触发一次 */
    if (motion_hold_ticks < MOTION_TEST_HOLD_TICKS) {
        motion_hold_ticks++;
    }
    if ((motion_hold_ticks >= MOTION_TEST_HOLD_TICKS)
        && (motion_hold_fired == 0U)) {
        motion_hold_fired = 1U;
        MotionTest_HoldAction(key);
    }
}

/* ===================== 调度 ===================== */

void MotionTest_Init(void)
{
    motion_mode = MOTION_MODE_IDLE;
    motion_speed_x10 = MOTION_TEST_SPEED_X10;
    motion_key_last = KEY_NONE;
    motion_hold_ticks = 0U;
    motion_hold_fired = 0U;
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
