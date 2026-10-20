/**
 * @file    heading.h
 * @brief   航向角环（外环）+ 编码器里程计
 *
 * ============================================================================
 *  为什么用里程计算航向
 * ----------------------------------------------------------------------------
 *  这台车上的 IMU601 只剩"发指令"的代码，姿态帧解析在本工程历史上就不存在
 *  （git 里最早的 imu601.c 也只有发送），拿不到真实 yaw。所以航向由四路
 *  编码器的位移积分得到：
 *
 *      d_vx  = ( LF + RF + LR + RR) / 4      前进位移
 *      d_vy  = ( LF - RF - LR + RR) / 4      左移位移
 *      d_rot = (-LF + RF - LR + RR) / 4      转动位移
 *      航向角 += d_rot * 360 / (π * 左右轮距)
 *
 *  四个方向的直线之所以都能保持航向，就是因为 d_rot 与走哪个方向无关：
 *  前进、后退、左移、右移时只要车身在转，这个量就会变。
 *
 *  局限（必须知道）：轮子打滑时里程计是被"骗"的——它算出的是轮子走过的
 *  距离差，不是车身真实转角。打滑严重时角度环会失效。要根治得用陀螺：
 *  见下面 Heading_InputExternalYaw() 的接入口。
 *
 * ============================================================================
 *  串级结构
 * ----------------------------------------------------------------------------
 *      角度环（本文件，10ms）：航向误差 -> wz 修正量（0.1cm/s）
 *          ↓ 叠加到运动层的 wz 上
 *      速度环（wheel_speed_control.c）：四个轮子的目标速度 -> PWM
 */
#ifndef __HEADING_H
#define __HEADING_H

#include "main.h"

/* 里程计累计量（只增不减，Heading_Reset()/上电后归零） */
extern volatile int32_t g_odom_vx_x100;      /* 累计前进位移，0.01cm */
extern volatile int32_t g_odom_vy_x100;      /* 累计左移位移，0.01cm */

void Heading_Init(void);

/* 10ms 中断里调用：先积分里程计，再算角度环输出 */
void Heading_Update10ms(void);

/* ===================== 角度环控制 ===================== */
/* 以当前航向为目标，开始保持直线（运动层在直行指令开始时调用） */
void Heading_Hold(void);

/* 关闭角度环（旋转、停车时调用），修正量清零 */
void Heading_Release(void);

uint8_t Heading_IsHolding(void);

/* 指定目标航向（0.1°，逆时针为正），可用于走固定角度 */
void Heading_SetTargetX10(int32_t yaw_01deg);

int32_t Heading_GetTargetX10(void);      /* 目标航向，0.1° */
int32_t Heading_GetYawX10(void);         /* 当前航向，0.1° */
int32_t Heading_GetErrorX10(void);       /* 航向误差，0.1° */
int32_t Heading_GetCorrectionX10(void);  /* 角度环输出（叠加到 wz），0.1cm/s */

/* ===================== 外部航向源（接 IMU 用） ===================== */
/*
 * 如果以后拿回 IMU601 的姿态帧格式，在收到一帧时调用：
 *      Heading_InputExternalYaw(yaw_01deg, 1);
 * 之后航向就用外部值，不再积分编码器；连续 200ms 没有新样例会退回里程计。
 *      Heading_InputExternalYaw(0, 0);   显式退回里程计
 */
void Heading_InputExternalYaw(int32_t yaw_01deg, uint8_t valid);

/* 0 = 里程计，1 = 外部（IMU） */
uint8_t Heading_GetSource(void);

/* 里程计清零（换场地/重新对齐时用） */
void Heading_Reset(void);

#endif
