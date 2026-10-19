#include "wheel_speed.h"
#include "wheel_speed_control.h"

/* 实测每厘米脉冲数 x100：LF、RF、LR、RR。 */
static const int32_t wheel_ppc_x100[ENCODER_NUM] = {
    7851L,
    7905L,
    7825L,
    7562L
};

static int32_t wheel_last_count[ENCODER_NUM] = {0, 0, 0, 0};

volatile int32_t g_wheel_delta[ENCODER_NUM] = {0, 0, 0, 0};
volatile int32_t g_wheel_speed_raw_x10[ENCODER_NUM] = {0, 0, 0, 0};
volatile int32_t g_wheel_speed_x10[ENCODER_NUM] = {0, 0, 0, 0};
volatile uint32_t g_wheel_speed_sample_count = 0U;

static int32_t WheelSpeed_GetDelta(Encoder_ID id, int32_t now, int32_t last)
{
    if ((id == ENCODER_LF) || (id == ENCODER_RF)) {
        /* 硬件编码器为16位计数器，强制转回int16_t可正确处理回绕。 */
        return (int32_t)(int16_t)(now - last);
    }
    return now - last;
}

void WheelSpeed_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i;

    __disable_irq();
    for (i = 0U; i < ENCODER_NUM; i++) {
        wheel_last_count[i] = Encoder_GetPhysicalCount((Encoder_ID)i);
        g_wheel_delta[i] = 0;
        g_wheel_speed_raw_x10[i] = 0;
        g_wheel_speed_x10[i] = 0;
    }
    g_wheel_speed_sample_count = 0U;
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

void WheelSpeed_Init(void)
{
    WheelSpeed_Reset();
}

void WheelSpeed_Update10ms(void)
{
    uint8_t i;

    for (i = 0U; i < ENCODER_NUM; i++) {
        Encoder_ID id = (Encoder_ID)i;
        int32_t now = Encoder_GetPhysicalCount(id);
        int32_t delta = WheelSpeed_GetDelta(id, now, wheel_last_count[i]);
        int32_t raw_x10;

        wheel_last_count[i] = now;
        g_wheel_delta[i] = delta;

        /*
         * Ts=10ms：速度(cm/s)x10 = delta * 100000 / PPC_x100。
         * 一阶低通：新值20%，旧值80%，降低单周期整数脉冲抖动。
         */
        raw_x10 = (delta * 100000L) / wheel_ppc_x100[i];
        g_wheel_speed_raw_x10[i] = raw_x10;
        g_wheel_speed_x10[i] =
            (g_wheel_speed_x10[i] * 4L + raw_x10) / 5L;
    }
    g_wheel_speed_sample_count++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        WheelSpeed_Update10ms();
        WheelSpeedControl_Update10ms();
    }
}
