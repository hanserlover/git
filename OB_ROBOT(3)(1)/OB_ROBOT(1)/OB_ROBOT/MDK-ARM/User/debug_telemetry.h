#ifndef __DEBUG_TELEMETRY_H
#define __DEBUG_TELEMETRY_H

#include "main.h"

/*
 * IMU 协议嗅探开关
 *   0 = 正常模式：USART1 按 VOFA+ FireWater 发 5 通道波形（目标 + 四轮速度）
 *   1 = 嗅探模式：USART1 按帧打印 IMU601(USART3) 收到的数据：
 *         [ms] OK-A L=12 AA 55 60 01 06 ..   按"第 5 字节=负载长度"校验通过
 *         [ms] OK-B L=..  AA 55 ..           任意长度中找到合法校验
 *         [ms] RAW? L=16  AA 55 ..           16 字节内校验不对（帧长估计错）
 *         [ms] RAWD L=12  ..                 长时间没抓到帧，原始字节兜底
 *       用途：没有模块手册时，抓一段就能反推上报帧格式（哪个字段随转动变）。
 *       注意：打开后 app.c 里不再调用 IMU601_Process()，
 *       否则数据会在主循环被提前读空、嗅探不到东西。
 */
#define DEBUG_IMU_SNIFFER     0

HAL_StatusTypeDef DebugTelemetry_Init(void);
void DebugTelemetry_Process(uint32_t now);
void DebugTelemetry_IRQHandler(void);

#endif
