#include "app.h"
#include "main.h"
#include "tim.h"
#include "move.h"
#include "encoder.h"
#include "sensor.h"
#include "key.h"
#include "oled.h"
#include "imu601.h"
#include "heading.h"
#include "wheel_speed.h"
#include "wheel_speed_control.h"
#include "motion_test.h"
#include "debug_telemetry.h"

static uint32_t app_tick_last = 0U;

void App_Init(void)
{
    Move_Init();                 /* 内含 Motor_Init()，同时清掉运动指令状态 */
    Encoder_Init();
    WheelSpeed_Init();
    Heading_Init();              /* 里程计清零 + 角度环待命 */
    WheelSpeedControl_Init();
    Sensor_Init();
    Key_Init();
    OLED_Init();
    IMU601_Init();

    if (DebugTelemetry_Init() != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    app_tick_last = g_tick_10ms;
    MotionTest_Init();
}

void App_Run(void)
{
    uint32_t tick = g_tick_10ms;

    /*
     * 10ms 节拍由 TIM2 中断累加产生；按键消抖、界面、遥测全部放在主循环，
     * 中断里只做采样和闭环，符合 AGENTS.md 的约定。
     */
    if ((tick - app_tick_last) >= 1U) {
        app_tick_last = tick;
        MotionTest_ProcessKey(Key_Scan());
#if (DEBUG_IMU_SNIFFER == 0)
        IMU601_Process();   /* 正常模式：把 IMU 上报的数据读空即可 */
#endif
        /* 嗅探模式下不能在这里读空，原始字节由 DebugTelemetry_Process() 取走 */
    }

    MotionTest_ProcessTick(tick);
    DebugTelemetry_Process(HAL_GetTick());
}
