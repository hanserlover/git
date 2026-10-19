/**
 * @file    move.c
 * @brief   麦轮底盘运动层实现
 *
 * ★本次修改★
 *   1) 四向移动（前/后/左/右）与原地旋转全部集中在本文件；
 *   2) 加入四轮极性修正表，用于抵消电机安装方向不一致；
 *   3) 底层 motor.c 的引脚映射已按 "左前↔左后、右前↔右后" 对调。
 */
#include "move.h"
#include "motor.h"

/* ##########################################################################
 * #                          ★ 参 数 区 ★                                  #
 * ########################################################################## */

/* ------------------------------------------------------------------------
 * 四轮极性修正：0 = 不反相，1 = 反相
 * ------------------------------------------------------------------------
 * 电机六根线整体互换后，某个轮子的物理转向可能仍与其余轮相反。
 * 判据：调用 Move_Forward() 时，四个轮子必须朝同一方向转。
 * 哪个轮子反了，就把对应宏改成 1。
 *
 * 单个轮子的原始方向可用 motor.c 的 Motor_XX_Fwd()/Motor_XX_Rev() 直接验证。
 */
#define WHEEL_INVERT_RF      0       /* 左前 PC8/PC9  + PA10 */
#define WHEEL_INVERT_RR      1        /* 左后 PC2/PC3  + PA8  */
#define WHEEL_INVERT_LF      0        /* 右前 PC12/PB5 + PA11 */
#define WHEEL_INVERT_LR      1        /* 右后 PC4/PC5  + PB4  */

/* ------------------------------------------------------------------------
 * 默认速度（0 ~ MOTOR_PWM_MAX=3599），供 Move_XXX(speed) 传参参考
 * --------------------------------------------------------------------- */
#define MOVE_SPEED_DEFAULT   900
#define MOVE_SPEED_TURN      800

/* ##########################################################################
 * #                          参数区结束                                     #
 * ########################################################################## */

/* 单轮限幅 */
static int16_t Move_Limit(int32_t v)
{
    if (v > (int32_t)MOTOR_PWM_MAX) {
        return (int16_t)MOTOR_PWM_MAX;
    }
    if (v < -(int32_t)MOTOR_PWM_MAX) {
        return (int16_t)(-(int32_t)MOTOR_PWM_MAX);
    }
    return (int16_t)v;
}

/* 单轮下发：叠加极性修正 */
static void Move_Wheel(Motor_ID id, int32_t v, uint8_t invert)
{
    int32_t s = (invert != 0U) ? (-v) : v;

    Motor_SetSpeed(id, Move_Limit(s));
}

/**
 * @brief  四轮速度直接下发（各轮已是"物理前进为正"的量）
 */
static void Move_Wheels(int32_t lf, int32_t rf, int32_t lr, int32_t rr)
{
    Move_Wheel(MOTOR_LF, lf, (uint8_t)WHEEL_INVERT_LF);
    Move_Wheel(MOTOR_RF, rf, (uint8_t)WHEEL_INVERT_RF);
    Move_Wheel(MOTOR_LR, lr, (uint8_t)WHEEL_INVERT_LR);
    Move_Wheel(MOTOR_RR, rr, (uint8_t)WHEEL_INVERT_RR);
}

/* ===================== 初始化 / 停止 ===================== */

void Move_Init(void)
{
    Motor_Init();
}

void Move_Stop(void)
{
    Motor_SetSpeed(MOTOR_LF, 0);
    Motor_SetSpeed(MOTOR_RF, 0);
    Motor_SetSpeed(MOTOR_LR, 0);
    Motor_SetSpeed(MOTOR_RR, 0);
}

/* ===================== 底盘速度解算 ===================== */

/**
 * @brief  麦轮逆运动学 + 极性修正
 * @param  vx 前进(正) / vy 左移(正) / wz 左旋(正)
 */
void Move_SetVelocity(int16_t vx, int16_t vy, int16_t wz)
{
    Move_Wheels((int32_t)vx + vy - wz,      /* LF */
                (int32_t)vx - vy + wz,      /* RF */
                (int32_t)vx - vy - wz,      /* LR */
                (int32_t)vx + vy + wz);     /* RR */
}

/* ===================== 四向移动 ===================== */

/* 前进：四轮同向 */
void Move_Forward(int16_t speed)
{
    Move_Wheels(speed, speed, speed, speed);
}

/* 后退：四轮同向反向 */
void Move_Backward(int16_t speed)
{
    Move_Wheels(-(int32_t)speed, -(int32_t)speed,
                -(int32_t)speed, -(int32_t)speed);
}

/* 左移：LF/RR 正，RF/LR 反 */
void Move_Left(int16_t speed)
{
    Move_Wheels(speed, -(int32_t)speed, -(int32_t)speed, speed);
}

/* 右移：LF/RR 反，RF/LR 正 */
void Move_Right(int16_t speed)
{
    Move_Wheels(-(int32_t)speed, speed, speed, -(int32_t)speed);
}

/* 原地左转：左侧反、右侧正 */
void Move_RotateLeft(int16_t speed)
{
    Move_Wheels(-(int32_t)speed, speed, -(int32_t)speed, speed);
}

/* 原地右转：左侧正、右侧反 */
void Move_RotateRight(int16_t speed)
{
    Move_Wheels(speed, -(int32_t)speed, speed, -(int32_t)speed);
}
