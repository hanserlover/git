#include <stdio.h>
#include "debug_telemetry.h"
#include "imu601.h"
#include "wheel_speed.h"
#include "wheel_speed_control.h"

/*
 * 单向调试串口：PA9=USART1_TX -> DAP RX，115200 8N1。
 * PA10仍由TIM1_CH3输出左前轮PWM，禁止连接DAP TX。
 */
#define TELEMETRY_PERIOD_MS       20U
#define TELEMETRY_BUFFER_SIZE     128U

#if (DEBUG_IMU_SNIFFER != 0)
/* 嗅探模式：按帧解析 + 原始字节兜底 */
#define SNIFF_PERIOD_MS           10U
#define SNIFF_READ_MAX            32U
#define SNIFF_FRAME_MAX           16U
#define SNIFF_BUFFER_SIZE         96U
#define SNIFF_RAW_TICKS           50U   /* 没抓到帧时每 500ms 打一行原始字节 */
#endif

static UART_HandleTypeDef debug_uart;
static uint32_t telemetry_tick = 0U;
static char telemetry_buffer[TELEMETRY_BUFFER_SIZE];

#if (DEBUG_IMU_SNIFFER == 0)
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
#endif /* DEBUG_IMU_SNIFFER == 0 */

#if (DEBUG_IMU_SNIFFER != 0)
/*
 * IMU 原始字节嗅探 + 按帧解析
 * ------------------------------------------------------------------------
 * 指令帧（MCU->模块）的结构是已知的：
 *     AA 55 | 类型 0x60 | 子命令 | 负载长度 | 负载... | 校验和
 *     校验和 = 从第 3 个字节累加到校验和前一个字节，取低 8 位
 * 例如软复位 AA 55 60 12 00 72、yaw 校准 AA 55 60 14 04 85 0B B5 43 00。
 *
 * 模块主动上报（模块->MCU）的帧结构没有手册，所以这里做两件事：
 *   1) 抓帧：见到 AA 55（或 55 AA）就缓存，然后
 *        OK-A  按"第 5 字节 = 负载长度"判定帧长且校验和通过
 *        OK-B  任意长度里找校验和通过的那个
 *        RAW?  16 字节内没找到合法校验（帧长估计不对）
 *        RAWD  长时间没抓到帧，定期吐原始字节，用来发现真实帧头
 *  2) 每行打印 "[ms] 标记 L=帧长 十六进制字节"，直接看哪个字段随转动变化
 *
 * 用法：DEBUG_IMU_SNIFFER 改 1 后烧写，串口助手 115200 抓文本：
 *   静止抓 20 行 -> 原地转 90° 再抓 20 行 -> 对比字节
 */

/* 与指令帧同一条校验规则 */
static uint8_t DebugTelemetry_Checksum(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    uint8_t sum = 0U;

    for (i = 2U; i < (uint8_t)(len - 1U); i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return sum;
}

/* 打一行 "[ms] tag L=n AA 55 .."；上一行没发完就丢掉这一帧 */
static void DebugTelemetry_SendFrameLine(uint32_t now, const char *tag,
                                         const uint8_t *frame, uint8_t len)
{
    static char line[SNIFF_BUFFER_SIZE];
    static const char hex_digit[] = "0123456789ABCDEF";
    int length;
    uint8_t i;

    if (HAL_UART_GetState(&debug_uart) != HAL_UART_STATE_READY) {
        return;
    }

    length = sprintf(line, "[%lu] %s L=%u", (unsigned long)now, tag,
                     (unsigned int)len);
    for (i = 0U; (i < len) && ((length + 4) < (int)SNIFF_BUFFER_SIZE); i++) {
        line[length++] = ' ';
        line[length++] = hex_digit[(frame[i] >> 4) & 0x0FU];
        line[length++] = hex_digit[frame[i] & 0x0FU];
    }
    line[length++] = '\r';
    line[length++] = '\n';
    (void)HAL_UART_Transmit_IT(&debug_uart, (uint8_t *)line, (uint16_t)length);
}

static uint8_t  sniff_frame[SNIFF_FRAME_MAX];
static uint8_t  sniff_len = 0U;
static uint8_t  sniff_prev = 0U;
static uint8_t  sniff_has_prev = 0U;
static uint16_t sniff_idle_ticks = 0U;

static void DebugTelemetry_SniffImu(uint32_t now)
{
    uint8_t raw[SNIFF_READ_MAX];
    uint16_t count;
    uint16_t i;
    uint8_t byte;

    if ((now - telemetry_tick) < SNIFF_PERIOD_MS) {
        return;
    }
    telemetry_tick = now;

    count = IMU601_ReadRaw(raw, (uint16_t)SNIFF_READ_MAX);

    if (count == 0U) {
        if (sniff_idle_ticks < SNIFF_RAW_TICKS) {
            sniff_idle_ticks++;
        }
        return;
    }

    for (i = 0U; i < count; i++) {
        byte = raw[i];

        /* 帧头：AA 55 或 55 AA 都当成候选，长度和校验在后面判 */
        if ((sniff_has_prev != 0U)
            && (((sniff_prev == 0xAAU) && (byte == 0x55U))
                || ((sniff_prev == 0x55U) && (byte == 0xAAU)))) {
            sniff_frame[0] = sniff_prev;
            sniff_frame[1] = byte;
            sniff_len = 2U;
            sniff_has_prev = 0U;
            continue;
        }

        sniff_prev = byte;
        sniff_has_prev = 1U;

        if (sniff_len == 0U) {
            continue;               /* 还没见到帧头，先丢 */
        }

        sniff_frame[sniff_len] = byte;
        sniff_len++;

        /* 规则 A：第 5 字节是负载长度（与指令帧一致），总长 = 负载 + 6 */
        if (sniff_len >= 6U) {
            uint8_t total = (uint8_t)(sniff_frame[4] + 6U);

            if ((total >= 6U) && (total <= sniff_len)
                && (DebugTelemetry_Checksum(sniff_frame, total)
                    == sniff_frame[total - 1U])) {
                DebugTelemetry_SendFrameLine(now, "OK-A", sniff_frame, total);
                sniff_len = 0U;
                sniff_idle_ticks = 0U;
                continue;
            }
        }

        /* 规则 B：任意长度里找校验和能对上的（从长到短） */
        if (sniff_len >= 6U) {
            uint8_t total;
            uint8_t found = 0U;

            for (total = sniff_len; total >= 6U; total--) {
                if (DebugTelemetry_Checksum(sniff_frame, total)
                    == sniff_frame[total - 1U]) {
                    found = total;
                    break;
                }
            }
            if (found != 0U) {
                DebugTelemetry_SendFrameLine(now, "OK-B", sniff_frame, found);
                sniff_len = 0U;
                sniff_idle_ticks = 0U;
                continue;
            }
        }

        if (sniff_len >= SNIFF_FRAME_MAX) {
            DebugTelemetry_SendFrameLine(now, "RAW?", sniff_frame,
                                         (uint8_t)SNIFF_FRAME_MAX);
            sniff_len = 0U;
            sniff_idle_ticks = 0U;
        }
    }

    /* 兜底：一直没抓到合法帧，就定期把原始字节吐出来看帧头到底长什么样 */
    if ((sniff_len == 0U) && (sniff_idle_ticks >= SNIFF_RAW_TICKS)) {
        sniff_idle_ticks = 0U;
        DebugTelemetry_SendFrameLine(now, "RAWD", raw,
                                     (uint8_t)((count > 12U) ? 12U : count));
    }
}
#endif /* DEBUG_IMU_SNIFFER */

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
#if (DEBUG_IMU_SNIFFER != 0)
    DebugTelemetry_SniffImu(now);
#else
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
#endif /* DEBUG_IMU_SNIFFER */
}

void DebugTelemetry_IRQHandler(void)
{
    HAL_UART_IRQHandler(&debug_uart);
}
