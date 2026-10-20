/**
 * @file    move.c
 * @brief   麦轮底盘运动层：前后左右移动 + 原地旋转 + 加减速斜坡 + 定时停车
 *
 * ★本次修改★
 *   1) 从"直接下发 PWM 的开环层"升级成"下发目标速度的闭环指令层"，
 *      速度单位 0.1 cm/s，前后左右都走 speed_control 的 PI + 前馈；
 *   2) 加入加减速斜坡：目标按 MOVE_ACCEL_X10_PER_TICK 逼近，
 *      前进↔后退切换时会先过零再反向，避免电流冲击和齿轮打齿；
 *   3) 支持定时运动（Move_DriveFor）与平滑停车（Move_Halt）；
 *   4) 全部计算放在 TIM2 的 10ms 中断里（Move_Update10ms），
 *      调用方接口非阻塞，可以直接放进按键/状态机里。
 *
 * 分层关系：
 *   move.c（本文件，运动指令 / 斜坡）
 *      ↓ 四个轮子的目标速度（0.1 cm/s，带符号）
 *   wheel_speed_control.c（PI + 前馈闭环）
 *      ↓ 四个轮子的 PWM（带符号）
 *   motor.c（TB6612 方向 + PWM）
 */
#include "move.h"
#include "motor.h"
#include "encoder.h"
#include "wheel_speed.h"
#include "wheel_speed_control.h"

/* ##########################################################################
 * #                          ★ 参 数 区 ★                                  #
 * ########################################################################## */

/* ------------------------------------------------------------------------
 * 四轮极性修正：0 = 不反相，1 = 反相
 * ------------------------------------------------------------------------
 * 判据：调用 Move_Forward() 时，四个轮子必须朝同一方向转。
 * 哪个轮子反了，就把对应宏改成 1。
 * 单个轮子的原始方向可用 motor.c 的 Motor_XX_Fwd()/Motor_XX_Rev() 直接验证。
 */
#define WHEEL_INVERT_LF      0        /* 左前 PC8/PC9  + PA10 (TIM1_CH3) */
#define WHEEL_INVERT_RF      0        /* 右前 PC4/PC5  + PB4  (TIM3_CH1) */
#define WHEEL_INVERT_LR      1        /* 左后 PC2/PC3  + PA8  (TIM1_CH1) */
#define WHEEL_INVERT_RR      1        /* 右后 PC12/PB5 + PA11 (TIM1_CH4) */

/* ------------------------------------------------------------------------
 * 速度与斜坡参数（单位 0.1 cm/s）
 * ------------------------------------------------------------------------
 * MOVE_SPEED_X10_MAX       单轮目标上限，闭环输出限幅 1500 PWM 约对应 30cm/s
 * MOVE_ACCEL_X10_PER_TICK  每 10ms 允许变化的速度量：8 => 80 cm/s²
 *                          10cm/s 大约 13 个控制周期（130ms）加到满
 * MOVE_SETTLE_TICKS        停稳后继续保持零目标的时间，之后关闭闭环
 */
#define MOVE_SPEED_X10_MAX          250L
#define MOVE_ACCEL_X10_PER_TICK     8L
#define MOVE_SETTLE_TICKS           10U

/* ------------------------------------------------------------------------
 * 直线行程纠偏（专治"四个轮速相等、车身却慢慢跑偏"）
 * ------------------------------------------------------------------------
 * 速度环只能保证每个轮子跟住自己的目标速度，但轮子打滑、麦轮辊子的侧向力、
 * 重心偏置都会让车身一点点偏航，速度环看不见这个偏差（它不测航向）。
 * 这里改用位置量：把左右两侧的累计行程相减，哪边走得多就往回转一点。
 * 这个偏差会随时间累加，所以哪怕稳态差速只有百分之几也能被纠回来。
 *
 *   MOVE_STRAIGHT_ENABLE        1=启用，0=关闭
 *   MOVE_STRAIGHT_KP            每 1cm 左右行程差对应的修正量（0.1cm/s）
 *                               30 => 1cm 差给 0.3cm/s 的旋转修正
 *   MOVE_STRAIGHT_DEADBAND_X100 死区，0.01cm 单位：30 => 0.3cm 以内不修
 *   MOVE_STRAIGHT_WZ_LIMIT_X10  修正量上限（0.1cm/s），防止打滑时猛打方向
 *   MOVE_STRAIGHT_MIN_SPEED_X10 车速太慢时不修正（原地附近没有意义）
 *
 * 只在"纯前进/后退"时生效：横移和旋转时左右行程本来就不相等。
 */
#define MOVE_STRAIGHT_ENABLE         1
#define MOVE_STRAIGHT_KP             30L
#define MOVE_STRAIGHT_DEADBAND_X100  30L
#define MOVE_STRAIGHT_WZ_LIMIT_X10   30L
#define MOVE_STRAIGHT_MIN_SPEED_X10  20L

/* ##########################################################################
 * #                          参数区结束                                     #
 * ########################################################################## */

/* 单轮 PWM 限幅 */
static int16_t Move_Limit(int32_t value)
{
    if (value > (int32_t)MOTOR_PWM_MAX) {
        return (int16_t)MOTOR_PWM_MAX;
    }
    if (value < -(int32_t)MOTOR_PWM_MAX) {
        return (int16_t)(-(int32_t)MOTOR_PWM_MAX);
    }
    return (int16_t)value;
}

/* 单轮 PWM 下发：叠加极性修正 */
static void Move_Wheel(Motor_ID id, int32_t value, uint8_t invert)
{
    int32_t speed = (invert != 0U) ? (-value) : value;

    Motor_SetSpeed(id, Move_Limit(speed));
}

/* 四轮 PWM 直接下发（各轮已是"物理前进为正"的量） */
static void Move_Wheels(int32_t lf, int32_t rf, int32_t lr, int32_t rr)
{
    Move_Wheel(MOTOR_LF, lf, (uint8_t)WHEEL_INVERT_LF);
    Move_Wheel(MOTOR_RF, rf, (uint8_t)WHEEL_INVERT_RF);
    Move_Wheel(MOTOR_LR, lr, (uint8_t)WHEEL_INVERT_LR);
    Move_Wheel(MOTOR_RR, rr, (uint8_t)WHEEL_INVERT_RR);
}

/* ===================== 运动指令状态 ===================== */

static volatile uint8_t  move_running = 0U;        /* 1 = 闭环在跑 */
static volatile int32_t  move_cmd_x10[3] = {0L, 0L, 0L};   /* 斜坡后的 vx/vy/wz */
static volatile int32_t  move_tgt_x10[3] = {0L, 0L, 0L};   /* 调用方给的目标 */
static volatile uint32_t move_duration_ticks = 0U;         /* 0 = 不限时 */
static volatile uint32_t move_elapsed_ticks = 0U;
static volatile uint8_t  move_settle_ticks = 0U;

/* 直线纠偏：本条指令开始时的"左-右"行程基准（0.01cm），以及当前偏差 */
#if (MOVE_STRAIGHT_ENABLE != 0)
static volatile int32_t move_straight_base_x100 = 0L;
#endif
static volatile int32_t move_straight_offset_x100 = 0L;

/* 前置声明：Move_Stop()/Move_Halt() 里要重置纠偏基准，实现在下面的纠偏小节 */
static void Move_StraightResetBase(void);

/* ===================== 初始化 / 停车 ===================== */

void Move_Init(void)
{
    Motor_Init();
    move_running = 0U;
    move_cmd_x10[0] = 0L;
    move_cmd_x10[1] = 0L;
    move_cmd_x10[2] = 0L;
    move_tgt_x10[0] = 0L;
    move_tgt_x10[1] = 0L;
    move_tgt_x10[2] = 0L;
    move_duration_ticks = 0U;
    move_elapsed_ticks = 0U;
    move_settle_ticks = 0U;
}

/* 立即断 PWM：先关掉速度闭环，再停电机，避免下一个控制周期被旧目标重新驱动 */
void Move_Stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    move_running = 0U;
    move_cmd_x10[0] = 0L;
    move_cmd_x10[1] = 0L;
    move_cmd_x10[2] = 0L;
    move_tgt_x10[0] = 0L;
    move_tgt_x10[1] = 0L;
    move_tgt_x10[2] = 0L;
    move_duration_ticks = 0U;
    move_elapsed_ticks = 0U;
    move_settle_ticks = 0U;

    /* 闭环也必须一起停：中断已被关掉，Abort 内部的 primask 恢复不会再开中断 */
    WheelSpeedControl_Abort();

    Move_StraightResetBase();

    Motor_SetSpeed(MOTOR_LF, 0);
    Motor_SetSpeed(MOTOR_RF, 0);
    Motor_SetSpeed(MOTOR_LR, 0);
    Motor_SetSpeed(MOTOR_RR, 0);
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

/* 平滑停车：目标清零，交给斜坡减速，停稳后中断里关闭闭环 */
void Move_Halt(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    move_tgt_x10[0] = 0L;
    move_tgt_x10[1] = 0L;
    move_tgt_x10[2] = 0L;
    move_duration_ticks = 0U;
    move_elapsed_ticks = 0U;
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }

    /* 闭环本来就没跑：把闭环和电机一起确认停掉，避免残留的旧目标 */
    if (move_running == 0U) {
        Move_Stop();
    }
    Move_StraightResetBase();
}

/* ===================== 底盘速度解算 ===================== */

static int32_t Move_ClampSpeed(int32_t value)
{
    if (value > MOVE_SPEED_X10_MAX) {
        return MOVE_SPEED_X10_MAX;
    }
    if (value < -MOVE_SPEED_X10_MAX) {
        return -MOVE_SPEED_X10_MAX;
    }
    return value;
}

/**
 * @brief  麦轮逆运动学：把底盘速度拆成四个轮子各自的目标速度
 * @param  vx 前进(正) / vy 左移(正) / wz 左旋(正)
 * @param  wheel 输出，下标沿用 Encoder_ID：LF、RF、LR、RR
 */
static void Move_Mix(int32_t vx, int32_t vy, int32_t wz,
                     int32_t wheel[ENCODER_NUM])
{
    wheel[ENCODER_LF] = Move_ClampSpeed(vx + vy - wz);
    wheel[ENCODER_RF] = Move_ClampSpeed(vx - vy + wz);
    wheel[ENCODER_LR] = Move_ClampSpeed(vx - vy - wz);
    wheel[ENCODER_RR] = Move_ClampSpeed(vx + vy + wz);
}

/* 速度斜坡：把 current 按每周期最多 step 的量逼向 target */
static int32_t Move_RampToward(int32_t current, int32_t target, int32_t step)
{
    int32_t diff = target - current;

    if (diff > step) {
        return current + step;
    }
    if (diff < -step) {
        return current - step;
    }
    return target;
}

/* ===================== 直线行程纠偏 ===================== */

#if (MOVE_STRAIGHT_ENABLE != 0)

/* 左右两侧累计行程之差：左 = LF+LR，右 = RF+RR（0.01cm，前进为正） */
static int32_t Move_SideDistanceDiff(void)
{
    int32_t left = g_wheel_distance_x100[ENCODER_LF]
                 + g_wheel_distance_x100[ENCODER_LR];
    int32_t right = g_wheel_distance_x100[ENCODER_RF]
                  + g_wheel_distance_x100[ENCODER_RR];

    return left - right;
}

/**
 * @brief  由左右行程差算出附加的 wz 修正量（0.1cm/s）
 * @note   左边多走 => 车身向右偏 => 需要左旋（wz > 0）把它掰回来。
 *         倒车时两侧累计行程都是负值，差值符号自动反向，
 *         所以前进/后退可以用同一套公式，不用分情况。
 */
static int32_t Move_StraightCorrection(void)
{
    int32_t offset = Move_SideDistanceDiff() - move_straight_base_x100;
    int32_t correction;

    if ((offset <= MOVE_STRAIGHT_DEADBAND_X100)
        && (offset >= -MOVE_STRAIGHT_DEADBAND_X100)) {
        return 0L;
    }

    correction = (offset * MOVE_STRAIGHT_KP) / 1000L;
    if (correction > MOVE_STRAIGHT_WZ_LIMIT_X10) {
        correction = MOVE_STRAIGHT_WZ_LIMIT_X10;
    } else if (correction < -MOVE_STRAIGHT_WZ_LIMIT_X10) {
        correction = -MOVE_STRAIGHT_WZ_LIMIT_X10;
    }
    return correction;
}

#endif /* MOVE_STRAIGHT_ENABLE */

/* 记录本周期的行程基准/偏差，同时把"新指令从零开始"这件事做掉 */
static void Move_StraightResetBase(void)
{
    move_straight_offset_x100 = 0L;
#if (MOVE_STRAIGHT_ENABLE != 0)
    move_straight_base_x100 = Move_SideDistanceDiff();
#endif
}

/* ===================== 指令下发 ===================== */

/**
 * @brief  四个方向的公共入口
 * @note   已经在动时只改目标，斜坡接着用当前速度，指令可以无缝切换；
 *         从静止起步时把斜坡起点清零并启动闭环。
 */
static void Move_StartCommand(int32_t vx_x10, int32_t vy_x10, int32_t wz_x10,
                              uint32_t duration_ms)
{
    uint32_t primask;

    vx_x10 = Move_ClampSpeed(vx_x10);
    vy_x10 = Move_ClampSpeed(vy_x10);
    wz_x10 = Move_ClampSpeed(wz_x10);

    primask = __get_PRIMASK();
    __disable_irq();
    move_tgt_x10[0] = vx_x10;
    move_tgt_x10[1] = vy_x10;
    move_tgt_x10[2] = wz_x10;
    move_elapsed_ticks = 0U;
    move_settle_ticks = 0U;
    Move_StraightResetBase();      /* 新指令：左右行程差从当前点重新算 */
    move_duration_ticks = 0U;
    if (duration_ms != 0U) {
        /* 少于一个控制周期也按 1 个周期算，避免"定时"变成"不限时" */
        move_duration_ticks = duration_ms / 10U;
        if (move_duration_ticks == 0U) {
            move_duration_ticks = 1U;
        }
    }

    if (move_running == 0U) {
        move_cmd_x10[0] = 0L;
        move_cmd_x10[1] = 0L;
        move_cmd_x10[2] = 0L;
        move_running = 1U;
        /* 以零目标启动，本轮之后的每个周期由 Move_Update10ms 抬升目标 */
        WheelSpeedControl_StartVector(0L, 0L, 0L, 0L);
    }
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

void Move_Drive(int32_t vx_x10, int32_t vy_x10, int32_t wz_x10)
{
    Move_StartCommand(vx_x10, vy_x10, wz_x10, 0U);
}

void Move_DriveFor(int32_t vx_x10, int32_t vy_x10, int32_t wz_x10,
                   uint32_t duration_ms)
{
    Move_StartCommand(vx_x10, vy_x10, wz_x10, duration_ms);
}

/* ===================== 四向移动 ===================== */

/* 前进：四轮同向 */
void Move_Forward(int32_t speed_x10)
{
    Move_StartCommand(speed_x10, 0L, 0L, 0U);
}

/* 后退：四轮同向反向 */
void Move_Backward(int32_t speed_x10)
{
    Move_StartCommand(-speed_x10, 0L, 0L, 0U);
}

/* 左移：LF/RR 正，RF/LR 反 */
void Move_Left(int32_t speed_x10)
{
    Move_StartCommand(0L, speed_x10, 0L, 0U);
}

/* 右移：LF/RR 反，RF/LR 正 */
void Move_Right(int32_t speed_x10)
{
    Move_StartCommand(0L, -speed_x10, 0L, 0U);
}

/* 原地左转：左侧反、右侧正 */
void Move_RotateLeft(int32_t speed_x10)
{
    Move_StartCommand(0L, 0L, speed_x10, 0U);
}

/* 原地右转：左侧正、右侧反 */
void Move_RotateRight(int32_t speed_x10)
{
    Move_StartCommand(0L, 0L, -speed_x10, 0U);
}

/* ===================== 底层 PWM 直通 ===================== */

void Move_SetWheelSpeeds(int16_t lf, int16_t rf, int16_t lr, int16_t rr)
{
    Move_Wheels(lf, rf, lr, rr);
}

/* ===================== 状态查询 ===================== */

uint8_t Move_IsMoving(void)
{
    return move_running;
}

void Move_GetCommand(int32_t *vx_x10, int32_t *vy_x10, int32_t *wz_x10)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (vx_x10 != 0) {
        *vx_x10 = move_cmd_x10[0];
    }
    if (vy_x10 != 0) {
        *vy_x10 = move_cmd_x10[1];
    }
    if (wz_x10 != 0) {
        *wz_x10 = move_cmd_x10[2];
    }
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

/* 当前"左-右"累计行程偏差（0.01cm），给界面/调试看走偏了多少 */
int32_t Move_GetStraightOffsetX100(void)
{
    return move_straight_offset_x100;
}

/* ===================== 10ms 斜坡 + 目标下发 ===================== */

void Move_Update10ms(void)
{
    int32_t wheel[ENCODER_NUM];
    int32_t vx;
    int32_t vy;
    int32_t wz;
    int32_t wz_out;
    uint8_t stopped;

    if (move_running == 0U) {
        return;
    }

    /* ---------- 1) 斜坡推进：每轴按 MOVE_ACCEL_X10_PER_TICK 逼近目标 ---------- */
    vx = Move_RampToward(move_cmd_x10[0], move_tgt_x10[0],
                         MOVE_ACCEL_X10_PER_TICK);
    vy = Move_RampToward(move_cmd_x10[1], move_tgt_x10[1],
                         MOVE_ACCEL_X10_PER_TICK);
    wz = Move_RampToward(move_cmd_x10[2], move_tgt_x10[2],
                         MOVE_ACCEL_X10_PER_TICK);
    move_cmd_x10[0] = vx;
    move_cmd_x10[1] = vy;
    move_cmd_x10[2] = wz;

    /*
     * ---------- 2) 直线行程纠偏 ----------
     * 只在纯前进/后退时叠加旋转修正：横移时左右行程本来就不相等，
     * 车太慢时也不修（0 附近没有意义，反而容易抖）。
     */
    wz_out = wz;
#if (MOVE_STRAIGHT_ENABLE != 0)
    if ((vy == 0L) && (wz == 0L)
        && ((vx >= MOVE_STRAIGHT_MIN_SPEED_X10)
            || (vx <= -MOVE_STRAIGHT_MIN_SPEED_X10))) {
        move_straight_offset_x100 =
            Move_SideDistanceDiff() - move_straight_base_x100;
        wz_out = wz + Move_StraightCorrection();
    } else {
        move_straight_offset_x100 = 0L;
    }
#endif

    Move_Mix(vx, vy, wz_out, wheel);

    /* ---------- 2) 停稳判定：四轮目标都到 0 后维持几周期再关闭环 ---------- */
    stopped = ((wheel[ENCODER_LF] == 0L) && (wheel[ENCODER_RF] == 0L)
            && (wheel[ENCODER_LR] == 0L) && (wheel[ENCODER_RR] == 0L)) ? 1U : 0U;

    if (stopped != 0U) {
        if (move_settle_ticks < MOVE_SETTLE_TICKS) {
            move_settle_ticks++;
            WheelSpeedControl_SetTargets(0L, 0L, 0L, 0L);
        } else {
            move_running = 0U;
            WheelSpeedControl_Stop();
            return;
        }
    } else {
        move_settle_ticks = 0U;
        WheelSpeedControl_SetTargets(wheel[ENCODER_LF], wheel[ENCODER_RF],
                                     wheel[ENCODER_LR], wheel[ENCODER_RR]);
    }

    /* ---------- 3) 定时运动到期：本周期已按目标跑过，下周期起减速 ---------- */
    if (move_duration_ticks != 0U) {
        move_elapsed_ticks++;
        if (move_elapsed_ticks >= move_duration_ticks) {
            move_duration_ticks = 0U;
            move_elapsed_ticks = 0U;
            move_tgt_x10[0] = 0L;
            move_tgt_x10[1] = 0L;
            move_tgt_x10[2] = 0L;
        }
    }
}
