#include "app.h"
#include "main.h"
#include "tim.h"
#include "motor.h"
#include "encoder.h"
#include "sensor.h"
#include "key.h"
#include "oled.h"
#include "imu601.h"
#include "wheel_speed.h"
#include "wheel_speed_control.h"
#include "speed_control_test.h"

static uint8_t app_last_key = KEY_NONE;
static uint32_t app_tick_10ms = 0U;

void App_Init(void)
{
    Motor_Init();
    Encoder_Init();
    WheelSpeed_Init();
    WheelSpeedControl_Init();
    Sensor_Init();
    Key_Init();
    OLED_Init();
    IMU601_Init();

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    SpeedControlTest_Init();
}

void App_Run(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - app_tick_10ms) >= 10U) {
        uint8_t key;

        app_tick_10ms = now;
        key = Key_Scan();
        if (key == KEY_NONE) {
            app_last_key = KEY_NONE;
        } else if (app_last_key == KEY_NONE) {
            app_last_key = key;
            SpeedControlTest_HandleKey(key);
        }

        IMU601_Process();
    }

    SpeedControlTest_Process(now);
}
