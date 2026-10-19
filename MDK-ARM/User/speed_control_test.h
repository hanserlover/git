#ifndef __SPEED_CONTROL_TEST_H
#define __SPEED_CONTROL_TEST_H

#include "main.h"

void SpeedControlTest_Init(void);
void SpeedControlTest_HandleKey(uint8_t key);
void SpeedControlTest_Process(uint32_t now);

#endif
