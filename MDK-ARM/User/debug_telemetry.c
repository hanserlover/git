#include <stdio.h>
#include "debug_telemetry.h"
#include "wheel_speed.h"
#include "wheel_speed_control.h"

/*
 * 单向调试串口：PA9=USART1_TX -> DAP RX，115200 8N1。
 * PA10仍由TIM1_CH3输出左前轮PWM，禁止连接DAP TX。
 */
#define TELEMETRY_PERIOD_MS       100U
#define TELEMETRY_BUFFER_SIZE     128U

static UART_HandleTypeDef debug_uart;
static uint32_t telemetry_tick = 0U;
static uint8_t telemetry_header_sent = 0U;
static char telemetry_buffer[TELEMETRY_BUFFER_SIZE];

static const char telemetry_header[] =
    "ms,run,target_x10,lf_x10,rf_x10,lr_x10,rr_x10,"
    "pwm_lf,pwm_rf,pwm_lr,pwm_rr\r\n";

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
    telemetry_header_sent = 0U;
    return HAL_OK;
}

void DebugTelemetry_Process(uint32_t now)
{
    int32_t speed[ENCODER_NUM];
    int16_t pwm[ENCODER_NUM];
    int32_t target_x10;
    uint8_t running;
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

    if (telemetry_header_sent == 0U) {
        if (HAL_UART_Transmit_IT(&debug_uart,
                (uint8_t *)telemetry_header,
                (uint16_t)(sizeof(telemetry_header) - 1U)) == HAL_OK) {
            telemetry_header_sent = 1U;
        }
        return;
    }

    /* 只在极短临界区内复制中断更新的数据，格式化和发送均在主循环。 */
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < ENCODER_NUM; i++) {
        speed[i] = g_wheel_speed_x10[i];
        pwm[i] = g_wheel_control_pwm[i];
    }
    target_x10 = WheelSpeedControl_GetTargetX10();
    running = WheelSpeedControl_IsRunning();
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }

    length = sprintf(telemetry_buffer,
        "%lu,%u,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d\r\n",
        (unsigned long)now,
        (unsigned int)running,
        (long)target_x10,
        (long)speed[ENCODER_LF],
        (long)speed[ENCODER_RF],
        (long)speed[ENCODER_LR],
        (long)speed[ENCODER_RR],
        (int)pwm[ENCODER_LF],
        (int)pwm[ENCODER_RF],
        (int)pwm[ENCODER_LR],
        (int)pwm[ENCODER_RR]);

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
