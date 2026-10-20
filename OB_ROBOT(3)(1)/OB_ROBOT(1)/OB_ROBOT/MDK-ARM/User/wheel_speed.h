#ifndef __WHEEL_SPEED_H
#define __WHEEL_SPEED_H

#include "encoder.h"

/*
 * 四轮顺序沿用 Encoder_ID：LF、RF、LR、RR。
 * 速度单位为 0.1 cm/s，正数表示车轮物理前进。
 */
extern volatile int32_t g_wheel_delta[ENCODER_NUM];
extern volatile int32_t g_wheel_speed_raw_x10[ENCODER_NUM];
extern volatile int32_t g_wheel_speed_x10[ENCODER_NUM];

/*
 * 累计行程，单位 0.01 cm，带符号（前进为正）。
 * 只在上电/ WheelSpeed_Reset() 后归零，move.c 的直线行程纠偏用它比较
 * 左右两侧"谁走得多"，是位置层的量，不是速度。
 */
extern volatile int32_t g_wheel_distance_x100[ENCODER_NUM];

extern volatile uint32_t g_wheel_speed_sample_count;

/* 10ms 节拍，由 TIM2 中断累加；主循环用它做非阻塞调度（不在中断里做 UI/打印） */
extern volatile uint32_t g_tick_10ms;

void WheelSpeed_Init(void);
void WheelSpeed_Reset(void);
void WheelSpeed_Update10ms(void);

/* 四轮累计行程的平均值（0.01cm，前进为正），给距离闭环用 */
int32_t WheelSpeed_GetAverageDistanceX100(void);

#endif
