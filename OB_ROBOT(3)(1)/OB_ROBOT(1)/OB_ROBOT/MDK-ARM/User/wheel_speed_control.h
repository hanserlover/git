/**
 * @file    wheel_speed_control.h
 * @brief   四轮速度闭环（PI + PWM 前馈），支持四轮各自带符号的目标速度
 *
 * 单位统一为 0.1 cm/s（100 = 10 cm/s），正值表示该车轮物理前进，
 * 负值表示反转。四轮顺序沿用 Encoder_ID：LF、RF、LR、RR。
 *
 * 给运动层（move.c）用的两个接口：
 *   WheelSpeedControl_StartVector()  清积分后启动（从静止起步）
 *   WheelSpeedControl_SetTargets()   运行中改目标，不清积分、不重启
 * 加减速斜坡每 10ms 更新一次目标，必须走 SetTargets，
 * 否则积分状态每 10ms 被清一次，闭环等于没做。
 */
#ifndef __WHEEL_SPEED_CONTROL_H
#define __WHEEL_SPEED_CONTROL_H

#include "encoder.h"

/* 每轮实际下发的 PWM（带符号）与当前目标速度，供串口波形和 OLED 观察 */
extern volatile int16_t g_wheel_control_pwm[ENCODER_NUM];
extern volatile int32_t g_wheel_target_x10[ENCODER_NUM];

void WheelSpeedControl_Init(void);

/* 四轮同速启动（旧接口；给负值即四轮反转） */
void WheelSpeedControl_Start(int32_t target_speed_x10);

/* 四轮各自带符号目标启动，单位 0.1 cm/s */
void WheelSpeedControl_StartVector(int32_t lf_x10, int32_t rf_x10,
                                   int32_t lr_x10, int32_t rr_x10);

/* 运行中更新四轮目标：不清积分、不重启闭环 */
void WheelSpeedControl_SetTargets(int32_t lf_x10, int32_t rf_x10,
                                  int32_t lr_x10, int32_t rr_x10);

void WheelSpeedControl_Stop(void);

/*
 * 只关闭环、清目标和积分，不碰电机 PWM。
 * 给 Move_Stop() 用：Move_Stop() 之后中断里的闭环必须真的停下来，
 * 否则下个 10ms 周期 PI 会按旧目标把电机重新驱动起来。
 * （不能直接调 WheelSpeedControl_Stop()，那会反过来调用 Move_Stop() 造成递归）
 */
void WheelSpeedControl_Abort(void);

uint8_t WheelSpeedControl_IsRunning(void);

/* 四轮目标中绝对值最大的那个（带符号），给波形 ch0 当“整车目标速度”用 */
int32_t WheelSpeedControl_GetTargetX10(void);

/* 指定车轮当前的目标速度 */
int32_t WheelSpeedControl_GetWheelTargetX10(Encoder_ID id);

void WheelSpeedControl_Update10ms(void);

#endif /* __WHEEL_SPEED_CONTROL_H */
