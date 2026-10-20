/**
 * @file    move.h
 * @brief   麦轮底盘运动层：前进 / 后退 / 左平移 / 右平移 / 原地旋转
 *
 * ★速度单位统一为 0.1 cm/s（100 = 10 cm/s）★
 *   正值表示车轮物理前进，负值表示反转。
 *
 * 底盘坐标：vx 前进为正，vy 左移为正，wz 左旋（逆时针）为正。
 * wz 直接按"轮速分量"给，不做几何换算：Move_RotateLeft(80) 表示
 * 左右两侧车轮各以 ±8 cm/s 反向滚动。
 *
 * 所有运动接口都是非阻塞的：真正的斜坡与闭环在 TIM2 的 10ms 中断里完成
 * （Move_Update10ms），调用方给个目标速度就能立刻返回，适合放进状态机。
 *
 * ★全部可调参数（四轮极性、速度上限、斜坡加速度）在 move.c 文件头部★
 */
#ifndef __MOVE_H
#define __MOVE_H

#include "main.h"

/* ===================== 初始化 / 停车 ===================== */
void Move_Init(void);

/* 立即断 PWM 并清掉运动指令（急停，会有一点机械冲击） */
void Move_Stop(void);

/* 按当前斜坡减速到 0，停稳后自动关闭闭环（正常停车用这个） */
void Move_Halt(void);

/* ===================== 四向移动（0.1 cm/s，非阻塞） ===================== */
void Move_Forward(int32_t speed_x10);      /* 前进 */
void Move_Backward(int32_t speed_x10);     /* 后退 */
void Move_Left(int32_t speed_x10);         /* 向左平移 */
void Move_Right(int32_t speed_x10);        /* 向右平移 */

/* ===================== 原地旋转（0.1 cm/s） ===================== */
void Move_RotateLeft(int32_t speed_x10);   /* 原地左转（逆时针） */
void Move_RotateRight(int32_t speed_x10);  /* 原地右转（顺时针） */

/* ===================== 任意方向组合 ===================== */
/* 三个分量可以同时给，例如 Move_Drive(100, 60, 0) = 边前进边左移 */
void Move_Drive(int32_t vx_x10, int32_t vy_x10, int32_t wz_x10);

/* 同上，但跑 duration_ms 毫秒后自动减速停车 */
void Move_DriveFor(int32_t vx_x10, int32_t vy_x10, int32_t wz_x10,
                   uint32_t duration_ms);

/*
 * 距离闭环：沿指令方向（vx 前后 / vy 左右）走满 distance_x100
 * （单位 0.01cm，10000 = 1m）后自动减速停车。
 * 行程取指令方向上的里程投影，所以横移也能用；
 * 停车前按当前速度预扣减速距离，落点在目标附近而不是"冲过头再减速"。
 * 同时带一个 3 倍预计耗时的安全超时，卡住不会一直转。
 */
void Move_DriveDistance(int32_t vx_x10, int32_t vy_x10, int32_t distance_x100);

/* 本条距离指令已经走了多少（0.01cm），供界面显示 */
int32_t Move_GetTravelX100(void);

/* ===================== 底层 PWM 直通（供速度闭环下发结果） ===================== */
void Move_SetWheelSpeeds(int16_t lf, int16_t rf, int16_t lr, int16_t rr);

/* ===================== 状态查询 ===================== */
uint8_t Move_IsMoving(void);               /* 1 = 正在闭环运行 */
void    Move_GetCommand(int32_t *vx_x10, int32_t *vy_x10, int32_t *wz_x10);

/* 内部：由 TIM2 10ms 中断在采样之后、闭环之前调用 */
void Move_Update10ms(void);

#endif /* __MOVE_H */
