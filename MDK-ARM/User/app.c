#include <stdio.h>
#include "app.h"
#include "main.h"
#include "tim.h"
#include "motor.h"
#include "encoder.h"
#include "sensor.h"
#include "key.h"
#include "oled.h"
#include "imu601.h"
#include "move.h"
#include "wheel_speed.h"

#define OLED_TEXT_SIZE               12U
#define FEEDFORWARD_TEST_TIME_MS     5000U
#define FEEDFORWARD_PWM_LEVELS       6U

static const uint16_t feedforward_pwm[FEEDFORWARD_PWM_LEVELS] = {
    300U, 500U, 700U, 900U, 1100U, 1300U
};

static uint8_t ff_pwm_index = 1U;
static uint8_t ff_running = 0U;
static uint8_t ff_last_key = KEY_NONE;
static uint32_t ff_start_tick = 0U;
static uint32_t app_tick_10ms = 0U;
static uint32_t app_tick_100ms = 0U;
static int32_t ff_result_speed_x10[ENCODER_NUM] = {0, 0, 0, 0};

static void App_FormatSpeed(char *text, int32_t speed_x10)
{
    uint32_t magnitude;
    char sign;

    if (speed_x10 < 0) {
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

static int32_t App_GetDisplaySpeed(Encoder_ID id)
{
    if (ff_running != 0U) {
        return g_wheel_speed_x10[id];
    }
    return ff_result_speed_x10[id];
}

static void FeedforwardTest_Show(void)
{
    char line[24];
    char lf[10];
    char rf[10];
    char lr[10];
    char rr[10];

    App_FormatSpeed(lf, App_GetDisplaySpeed(ENCODER_LF));
    App_FormatSpeed(rf, App_GetDisplaySpeed(ENCODER_RF));
    App_FormatSpeed(lr, App_GetDisplaySpeed(ENCODER_LR));
    App_FormatSpeed(rr, App_GetDisplaySpeed(ENCODER_RR));

    /* 先在RAM中完成整帧，再一次性刷新，避免“黑屏帧→内容帧”闪烁。 */
    OLED_ClearBuffer();
    sprintf(line, "FF PWM:%u %s",
            (unsigned int)feedforward_pwm[ff_pwm_index],
            (ff_running != 0U) ? "RUN" : "READY");
    OLED_ShowString(0, 0, line, OLED_TEXT_SIZE);

    sprintf(line, "LF:%s RF:%s", lf, rf);
    OLED_ShowString(0, 16, line, OLED_TEXT_SIZE);
    sprintf(line, "LR:%s RR:%s", lr, rr);
    OLED_ShowString(0, 32, line, OLED_TEXT_SIZE);

    if (ff_running != 0U) {
        OLED_ShowString(0, 48, "CENTER=STOP", OLED_TEXT_SIZE);
    } else {
        OLED_ShowString(0, 48, "UP/DN PWM CTR RUN", OLED_TEXT_SIZE);
    }
    OLED_Refresh();
}

static void FeedforwardTest_ClearResult(void)
{
    uint8_t i;

    for (i = 0U; i < ENCODER_NUM; i++) {
        ff_result_speed_x10[i] = 0;
    }
}

static void FeedforwardTest_Start(void)
{
    Encoder_ClearAll();
    WheelSpeed_Reset();
    FeedforwardTest_ClearResult();
    ff_running = 1U;
    ff_start_tick = HAL_GetTick();
    Move_Forward((int16_t)feedforward_pwm[ff_pwm_index]);
    FeedforwardTest_Show();
}

static void FeedforwardTest_Stop(void)
{
    uint8_t i;

    Move_Stop();
    for (i = 0U; i < ENCODER_NUM; i++) {
        ff_result_speed_x10[i] = g_wheel_speed_x10[i];
    }
    ff_running = 0U;
    FeedforwardTest_Show();
}

static void FeedforwardTest_HandleKey(uint8_t key)
{
    if (key == KEY_CENTER) {
        if (ff_running != 0U) {
            FeedforwardTest_Stop();
        } else {
            FeedforwardTest_Start();
        }
        return;
    }

    if (ff_running != 0U) {
        return;
    }

    if ((key == KEY_UP) && (ff_pwm_index + 1U < FEEDFORWARD_PWM_LEVELS)) {
        ff_pwm_index++;
        FeedforwardTest_ClearResult();
        FeedforwardTest_Show();
    } else if ((key == KEY_DOWN) && (ff_pwm_index > 0U)) {
        ff_pwm_index--;
        FeedforwardTest_ClearResult();
        FeedforwardTest_Show();
    }
}

void App_Init(void)
{
    Motor_Init();
    Encoder_Init();
    WheelSpeed_Init();
    Sensor_Init();
    Key_Init();
    OLED_Init();
    IMU601_Init();

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    Move_Stop();
    FeedforwardTest_ClearResult();
    FeedforwardTest_Show();
}

void App_Run(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - app_tick_10ms) >= 10U) {
        uint8_t key;

        app_tick_10ms = now;
        key = Key_Scan();
        if (key == KEY_NONE) {
            ff_last_key = KEY_NONE;
        } else if (ff_last_key == KEY_NONE) {
            ff_last_key = key;
            FeedforwardTest_HandleKey(key);
        }

        IMU601_Process();
    }

    if ((ff_running != 0U)
        && ((now - ff_start_tick) >= FEEDFORWARD_TEST_TIME_MS)) {
        FeedforwardTest_Stop();
    }

    if ((now - app_tick_100ms) >= 100U) {
        app_tick_100ms = now;
        FeedforwardTest_Show();
    }
}
