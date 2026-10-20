#ifndef __DEBUG_TELEMETRY_H
#define __DEBUG_TELEMETRY_H

#include "main.h"

/*
 * IMU 协议嗅探开关
 *   0 = 正常模式：USART1 按 VOFA+ FireWater 发 5 通道波形（目标 + 四轮速度）
 *   1 = 嗅探模式：USART1 把 IMU601(USART3) 收到的原始字节按 hex 打印，
 *       每行 "[ms] AA 55 .. .."，最多 12 字节一行。
 *       用途：没有模块手册时，抓一段 hex 反推上报帧格式。
 *       注意：打开后 app.c 里不再调用 IMU601_Process()，
 *       否则数据会在主循环被提前读空、嗅探不到东西。
 */
#define DEBUG_IMU_SNIFFER     0

HAL_StatusTypeDef DebugTelemetry_Init(void);
void DebugTelemetry_Process(uint32_t now);
void DebugTelemetry_IRQHandler(void);

#endif
