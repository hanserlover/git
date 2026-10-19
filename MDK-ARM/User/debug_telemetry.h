#ifndef __DEBUG_TELEMETRY_H
#define __DEBUG_TELEMETRY_H

#include "main.h"

HAL_StatusTypeDef DebugTelemetry_Init(void);
void DebugTelemetry_Process(uint32_t now);
void DebugTelemetry_IRQHandler(void);

#endif
