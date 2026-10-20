#include <stdio.h>
#include "speed_control_test.h"
#include "encoder.h"
#include "key.h"
#include "oled.h"
#include "wheel_speed.h"
#include "wheel_speed_control.h"

#define OLED_TEXT_SIZE                 12U
#define SPEED_TEST_TIME_MS             5000U
#define SPEED_DISPLAY_PERIOD_MS        100U
#define SPEED_TARGET_MIN_X10           50L
#define SPEED_TARGET_MAX_X10           200L
#define SPEED_TARGET_STEP_X10          10L

static int32_t speed_target_x10 = 100L;
static uint8_t speed_test_running = 0U;
static uint32_t speed_test_start_tick = 0U;
static uint32_t speed_display_tick = 0U;
static int32_t speed_result_x10[ENCODER_NUM] = {0L, 0L, 0L, 0L};

static void SpeedControlTest_FormatSpeed(char *text, int32_t speed_x10)
{
    uint32_t magnitude;
    char sign;

    if (speed_x10 < 0L) {
        sign = '-';
        magnitude = (uint32_t)(-speed_x10);
    } else {
        sign = '+';
        magnitude = (uint32_t)speed_x10;
    }

    sprintf(text, "%c%lu.%lu", sign,
            (unsigned long)(magnitude / 10U),
            (unsigned long)(magnitude % 10U));
}

static int32_t SpeedControlTest_GetDisplaySpeed(Encoder_ID id)
{
    if (speed_test_running != 0U) {
        return g_wheel_speed_x10[id];
    }
    return speed_result_x10[id];
}

static void SpeedControlTest_Show(void)
{
    char line[24];
    char lf[10];
    char rf[10];
    char lr[10];
    char rr[10];

    SpeedControlTest_FormatSpeed(lf,
        SpeedControlTest_GetDisplaySpeed(ENCODER_LF));
    SpeedControlTest_FormatSpeed(rf,
        SpeedControlTest_GetDisplaySpeed(ENCODER_RF));
    SpeedControlTest_FormatSpeed(lr,
        SpeedControlTest_GetDisplaySpeed(ENCODER_LR));
    SpeedControlTest_FormatSpeed(rr,
        SpeedControlTest_GetDisplaySpeed(ENCODER_RR));

    OLED_ClearBuffer();
    sprintf(line, "PI T:%lu.%lucm/s %s",
            (unsigned long)(speed_target_x10 / 10L),
            (unsigned long)(speed_target_x10 % 10L),
            (speed_test_running != 0U) ? "RUN" : "STOP");
    OLED_ShowString(0, 0, line, OLED_TEXT_SIZE);

    sprintf(line, "LF:%s RF:%s", lf, rf);
    OLED_ShowString(0, 16, line, OLED_TEXT_SIZE);
    sprintf(line, "LR:%s RR:%s", lr, rr);
    OLED_ShowString(0, 32, line, OLED_TEXT_SIZE);

    if (speed_test_running != 0U) {
        OLED_ShowString(0, 48, "AUTO5S CTR=STOP", OLED_TEXT_SIZE);
    } else {
        OLED_ShowString(0, 48, "UP/DN T CTR=START", OLED_TEXT_SIZE);
    }
    OLED_Refresh();
}

static void SpeedControlTest_ClearResult(void)
{
    uint8_t i;

    for (i = 0U; i < ENCODER_NUM; i++) {
        speed_result_x10[i] = 0L;
    }
}

static void SpeedControlTest_Start(void)
{
    Encoder_ClearAll();
    WheelSpeed_Reset();
    SpeedControlTest_ClearResult();
    speed_test_running = 1U;
    speed_test_start_tick = HAL_GetTick();
    WheelSpeedControl_Start(speed_target_x10);
    SpeedControlTest_Show();
}

static void SpeedControlTest_Stop(void)
{
    uint8_t i;

    WheelSpeedControl_Stop();
    for (i = 0U; i < ENCODER_NUM; i++) {
        speed_result_x10[i] = g_wheel_speed_x10[i];
    }
    speed_test_running = 0U;
    SpeedControlTest_Show();
}

void SpeedControlTest_Init(void)
{
    speed_test_running = 0U;
    speed_test_start_tick = 0U;
    speed_display_tick = HAL_GetTick();
    SpeedControlTest_ClearResult();
    WheelSpeedControl_Stop();
    SpeedControlTest_Show();
}

void SpeedControlTest_HandleKey(uint8_t key)
{
    if (key == KEY_CENTER) {
        if (speed_test_running != 0U) {
            SpeedControlTest_Stop();
        } else {
            SpeedControlTest_Start();
        }
        return;
    }

    if (speed_test_running != 0U) {
        return;
    }

    if ((key == KEY_UP)
        && (speed_target_x10 < SPEED_TARGET_MAX_X10)) {
        speed_target_x10 += SPEED_TARGET_STEP_X10;
        SpeedControlTest_ClearResult();
        SpeedControlTest_Show();
    } else if ((key == KEY_DOWN)
               && (speed_target_x10 > SPEED_TARGET_MIN_X10)) {
        speed_target_x10 -= SPEED_TARGET_STEP_X10;
        SpeedControlTest_ClearResult();
        SpeedControlTest_Show();
    }
}

void SpeedControlTest_Process(uint32_t now)
{
    if ((speed_test_running != 0U)
        && ((now - speed_test_start_tick) >= SPEED_TEST_TIME_MS)) {
        SpeedControlTest_Stop();
    }

    if ((now - speed_display_tick) >= SPEED_DISPLAY_PERIOD_MS) {
        speed_display_tick = now;
        SpeedControlTest_Show();
    }
}
