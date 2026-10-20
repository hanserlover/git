#include <stdio.h>
#include "debug_telemetry.h"
#include "wheel_speed.h"
#include "wheel_speed_control.h"

/*
 * 单向调试串口：PA9=USART1_TX -> DAP RX，115200 8N1。
 * PA10仍由TIM1_CH3输出左前轮PWM，禁止连接DAP TX。
 */
#define TELEMETRY_PERIOD_MS       20U
#define TELEMETRY_BUFFER_SIZE     128U

static UART_HandleTypeDef debug_uart;
static uint32_t telemetry_tick = 0U;
static char telemetry_buffer[TELEMETRY_BUFFER_SIZE];

static void DebugTelemetry_FormatX10(char *text, int32_t value_x10)
{
    uint32_t magnitude;

    if (value_x10 < 0L) {
        magnitude = (uint32_t)(-value_x10);
        sprintf(text, "-%lu.%lu",
                (unsigned long)(magnitude / 10U),
                (unsigned long)(magnitude % 10U));
    } else {
        magnitude = (uint32_t)value_x10;
        sprintf(text, "%lu.%lu",
                (unsigned long)(magnitude / 10U),
                (unsigned long)(magnitude % 10U));
    }
}

HAL_StatusTypeDef DebugTelemetry_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    HAL_StatusTypeDef status;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    debug_uart.Instance = USART1;
    debug_uart.Init.BaudRate = 115200;
    debug_uart.Init.WordLength = UART_WORDLENGTH_8B;
    debug_uart.Init.StopBits = UART_STOPBITS_1;
    debug_uart.Init.Parity = UART_PARITY_NONE;
    debug_uart.Init.Mode = UART_MODE_TX;
    debug_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    debug_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    status = HAL_UART_Init(&debug_uart);
    if (status != HAL_OK) {
        return status;
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    telemetry_tick = HAL_GetTick();
    return HAL_OK;
}

void DebugTelemetry_Process(uint32_t now)
{
    int32_t speed[ENCODER_NUM];
    int32_t target_x10;
    char target[16];
    char lf[16];
    char rf[16];
    char lr[16];
    char rr[16];
    uint32_t primask;
    uint8_t i;
    int length;

    if ((now - telemetry_tick) < TELEMETRY_PERIOD_MS) {
        return;
    }
    telemetry_tick = now;

    if (HAL_UART_GetState(&debug_uart) != HAL_UART_STATE_READY) {
        return;
    }

    /* 只在极短临界区内复制中断更新的数据，格式化和发送均在主循环。 */
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < ENCODER_NUM; i++) {
        speed[i] = g_wheel_speed_x10[i];
    }
    target_x10 = WheelSpeedControl_GetTargetX10();
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }

    DebugTelemetry_FormatX10(target, target_x10);
    DebugTelemetry_FormatX10(lf, speed[ENCODER_LF]);
    DebugTelemetry_FormatX10(rf, speed[ENCODER_RF]);
    DebugTelemetry_FormatX10(lr, speed[ENCODER_LR]);
    DebugTelemetry_FormatX10(rr, speed[ENCODER_RR]);

    /* VOFA+ FireWater：ch0=目标，ch1~ch4=LF、RF、LR、RR。 */
    length = sprintf(telemetry_buffer, "%s,%s,%s,%s,%s\n",
                     target, lf, rf, lr, rr);

    if ((length > 0) && ((uint32_t)length < TELEMETRY_BUFFER_SIZE)) {
        (void)HAL_UART_Transmit_IT(&debug_uart,
                                  (uint8_t *)telemetry_buffer,
                                  (uint16_t)length);
    }
}

void DebugTelemetry_IRQHandler(void)
{
    HAL_UART_IRQHandler(&debug_uart);
}
