/**
 * @file    move.h
 * @brief   麦轮底盘运动层：前后左右移动、横移、原地旋转
 *
 * 坐标约定：vx 前进为正；vy 左移为正；wz 逆时针(左旋)为正。
 * 四向移动函数已按 ★电机对调★ 后的映射实现，调用方无需关心里程细节。
 *
 * ★全部可调参数（四轮极性、默认速度）在 move.c 文件头部★
 */
#ifndef __MOVE_H
#define __MOVE_H

#include "main.h"

/* ===================== 初始化 / 停止 ===================== */
void Move_Init(void);
void Move_Stop(void);

/* ===================== 底盘速度（麦轮逆运动学 + 极性修正） ===================== */
void Move_SetWheelSpeeds(int16_t lf, int16_t rf, int16_t lr, int16_t rr);
void Move_SetVelocity(int16_t vx, int16_t vy, int16_t wz);

/* ===================== 四向移动（speed 范围 0 ~ MOTOR_PWM_MAX） ===================== */
void Move_Forward(int16_t speed);        /* 前进 */
void Move_Backward(int16_t speed);       /* 后退 */
void Move_Left(int16_t speed);           /* 向左平移 */
void Move_Right(int16_t speed);          /* 向右平移 */

/* ===================== 原地旋转 ===================== */
void Move_RotateLeft(int16_t speed);     /* 原地左转（逆时针） */
void Move_RotateRight(int16_t speed);    /* 原地右转（顺时针） */

#endif /* __MOVE_H */
