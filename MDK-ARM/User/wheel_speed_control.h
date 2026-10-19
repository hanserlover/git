#ifndef __WHEEL_SPEED_CONTROL_H
#define __WHEEL_SPEED_CONTROL_H

#include "encoder.h"

/* 四轮顺序沿用 Encoder_ID：LF、RF、LR、RR。 */
extern volatile int16_t g_wheel_control_pwm[ENCODER_NUM];

void WheelSpeedControl_Init(void);
void WheelSpeedControl_Start(int32_t target_speed_x10);
void WheelSpeedControl_Stop(void);
uint8_t WheelSpeedControl_IsRunning(void);
void WheelSpeedControl_Update10ms(void);

#endif
