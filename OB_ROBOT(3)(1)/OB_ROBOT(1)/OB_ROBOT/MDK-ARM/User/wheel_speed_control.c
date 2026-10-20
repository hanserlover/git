/**
 * @file    wheel_speed_control.c
 * @brief   四轮速度闭环：增量式 PI + 前馈 + 静摩擦补偿
 *
 * ★本次重写（针对"走不直"）★
 *   1) 前馈加静摩擦补偿：四个轮子在同一周期一起破静摩擦。
 *      原来只有 45*速度 的线性前馈，低速段 PWM 很小，谁先克服摩擦谁先转，
 *      起步那一瞬间车头就被拽偏了；加一个固定的摩擦补偿后四轮同步起动。
 *   2) PI 从"位置式"改成"增量式（速度式）"：
 *          u[k] = u[k-1] + Kp*(e[k]-e[k-1]) + Ki*e[k] + (FF[k]-FF[k-1])
 *      不再有独立的积分状态，因此不存在积分饱和，也不会出现"某个轮子的
 *      积分憋满、另一个还空着"这种左右不对称；目标一变，输出立刻跟着走。
 *   3) 输出带 ±1500 的双向限幅：减速和倒车时 PI 真的能"反向出力"，
 *      原来是 0~1500，倒车和刹车全靠前馈，稳态误差只能靠时间慢慢磨。
 *   4) 直线配平表改成"正值 = 该轮目标更快"，符号跟随行驶方向，
 *      前进时 RR 快 0.3cm/s，倒车时自动反向补偿。
 *   5) 目标突变（换挡/换方向）时按前馈重新起调，旧状态不会拖着新方向。
 *   6) 目标为 0 时改为"阻尼刹车"（反向 PWM 与实测速度成正比），
 *      停车确定、不往前溜、也不反向倒车；原来这种工况被前馈顶着溜一段。
 *
 * ★PID 参数保持你在直行时标定的那组（30 / 40 / 45）★
 *   换的是结构，不是手感；摩擦补偿 PWM 是新增量，见下面参数区。
 *
 * ★配套：车身"走不直"的慢性偏航由 move.c 的行程纠偏处理★
 *   速度环只能保证"每个轮子跟住自己的目标"，轮子打滑、辊子侧向力造成的
 *   车身偏航属于位置层问题，需要左右累计行程差来修正。
 */
#include "wheel_speed_control.h"
#include "motor.h"
#include "move.h"
#include "wheel_speed.h"

/* ##########################################################################
 * #                          ★ 参 数 区 ★                                  #
 * ########################################################################## */

/* ------------------------------------------------------------------------
 * 前馈：PWM = 速度(0.1cm/s) * FF_GAIN_X10 / 10 + 静摩擦补偿
 * ------------------------------------------------------------------------
 * FF_GAIN_X10      约 45 PWM/(cm/s)，来自 PWM500 附近的实测速度
 * FF_FRICTION_PWM  静摩擦补偿：目标不为 0 时按符号叠加的固定 PWM。
 *                  目的不是提速，而是让四个轮子"同时"越过静摩擦死区。
 *                  偏小 → 起步时慢的那个轮子拖后腿，车头偏；
 *                  偏大 → 低速时像"窜出去"，靠 PI 反向拉回来。
 *                  取值方法：把车架空，单轮给 0.1 的目标速度，
 *                  从 0 慢慢加这个值，直到四个轮子几乎同时转起来。
 */
#define WHEEL_FF_GAIN_X10             45L
#define WHEEL_FF_FRICTION_PWM         120L

/* ------------------------------------------------------------------------
 * PI 参数（增量式）
 * ------------------------------------------------------------------------
 *   P 项：KP_X10 * (误差 - 上次误差) / 10
 *   I 项：KI_X1000 * 误差 / 1000      每个 10ms 周期累加一次
 * 这组 30 / 40 是原直行标定值，先别动；走不直优先按这个顺序查：
 *   a) 摩擦补偿是否合适（起步偏）      WHEEL_FF_FRICTION_PWM（本文件）
 *   b) 角度环增益/死区（慢性偏、蛇行）  HEADING_KP / HEADING_KI（heading.c）
 *   c) 编码器 PPC 标定（距离对不上）    wheel_ppc_x100（wheel_speed.c）
 */
#define WHEEL_PI_KP_X10               30L
#define WHEEL_PI_KI_X1000             40L

/* 输出限幅（PWM）：±1500 对应约 ±30cm/s，留出了刹车和倒车的余量 */
#define WHEEL_CONTROL_PWM_LIMIT       1500L

/* 目标速度限幅（0.1cm/s）：25 cm/s 已覆盖小车实际需要 */
#define WHEEL_TARGET_X10_LIMIT        250L

/*
 * 目标跳变阈值：相邻两个控制周期的目标差超过它就按"无扰切换"重新起调
 * （输出直接置为前馈，误差历史清零）。加减速斜坡是 8/周期，不会触发；
 * 换方向、换模式这种大跳变会触发，避免旧工况的状态拖累新方向。
 */
#define WHEEL_TARGET_JUMP_X10         30L

/* 零速死区：|实测速度| 小于它就认为轮子已经停住，直接断输出，防止爬行 */
#define WHEEL_ZERO_SPEED_X10          2L

/*
 * 目标为 0 时的刹车方式：只做"阻尼刹车"（反向 PWM 与实测速度成正比），
 * 不再走 PI。好处是停车过程确定、不会因为前馈残留或积分憋着而往前溜，
 * 也不会反向倒车；原来靠 0~1500 的单向限幅，刹车完全靠时间磨。
 *   WHEEL_BRAKE_GAIN_X10  10cm/s 对应 -300 PWM
 *   WHEEL_BRAKE_PWM_LIMIT 刹车强度上限
 */
#define WHEEL_BRAKE_GAIN_X10          30L
#define WHEEL_BRAKE_PWM_LIMIT         800L

/* ------------------------------------------------------------------------
 * 直线配平表（单位 0.1cm/s，正值 = 该轮目标更快）
 * ------------------------------------------------------------------------
 * PPC 只表示编码器距离比例，轮胎直径、装配、轴承预紧的差异要靠这里补。
 * 调法：直行 3m，看车尾相对车头往哪边偏 →
 *       往右偏 说明左侧走得多，把左侧(或对侧)目标微调 +0.1cm/s；
 *       每次改 1~2 个 0.1cm/s，改完复测，别一次调太多。
 * 符号自动跟随行驶方向，倒车不会把补偿方向搞反。
 * 有了 move.c 的行程纠偏后，这张表只需要补"稳态差速"，可以调得很小。
 */
static const int32_t wheel_trim_x10[ENCODER_NUM] = {
    0L,     /* LF */
    0L,     /* RF */
    0L,     /* LR */
    3L      /* RR：原来修的"车辆向右偏" */
};

/* ##########################################################################
 * #                          参数区结束                                     #
 * ########################################################################## */

static volatile uint8_t wheel_control_running = 0U;
static volatile int32_t wheel_target_main_x10 = 0L;

/* 增量式 PI 的状态：上一周期的输出、误差、前馈、目标 */
static int32_t wheel_output_pwm[ENCODER_NUM] = {0L, 0L, 0L, 0L};
static int32_t wheel_error_prev_x10[ENCODER_NUM] = {0L, 0L, 0L, 0L};
static int32_t wheel_ff_prev[ENCODER_NUM] = {0L, 0L, 0L, 0L};
static int32_t wheel_target_prev_x10[ENCODER_NUM] = {0L, 0L, 0L, 0L};

volatile int16_t g_wheel_control_pwm[ENCODER_NUM] = {0, 0, 0, 0};
volatile int32_t g_wheel_target_x10[ENCODER_NUM] = {0L, 0L, 0L, 0L};

static int32_t WheelSpeedControl_Limit(int32_t value,
                                      int32_t minimum,
                                      int32_t maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

/* 目标 + 配平：配平量按行驶方向加符号 */
static int32_t WheelSpeedControl_EffectiveTarget(uint8_t index)
{
    int32_t target = g_wheel_target_x10[index];
    int32_t trim = wheel_trim_x10[index];

    if (trim == 0L) {
        return target;
    }
    if (target > 0L) {
        return target + trim;
    }
    if (target < 0L) {
        return target - trim;
    }
    return target;
}

/* 前馈：线性项 + 静摩擦补偿（目标为 0 时不补偿，保证真能停） */
static int32_t WheelSpeedControl_Feedforward(int32_t target_x10)
{
    if (target_x10 > 0L) {
        return (target_x10 * WHEEL_FF_GAIN_X10) / 10L + WHEEL_FF_FRICTION_PWM;
    }
    if (target_x10 < 0L) {
        return (target_x10 * WHEEL_FF_GAIN_X10) / 10L - WHEEL_FF_FRICTION_PWM;
    }
    return 0L;
}

/* 参数顺序固定为 LF、RF、LR、RR，内部换算成 Encoder_ID 下标 */
static void WheelSpeedControl_WriteTargets(int32_t lf_x10, int32_t rf_x10,
                                          int32_t lr_x10, int32_t rr_x10)
{
    int32_t target[ENCODER_NUM];
    int32_t main_x10 = 0L;
    int32_t main_magnitude = 0L;
    uint8_t i;

    target[ENCODER_LF] = WheelSpeedControl_Limit(
        lf_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);
    target[ENCODER_RF] = WheelSpeedControl_Limit(
        rf_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);
    target[ENCODER_LR] = WheelSpeedControl_Limit(
        lr_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);
    target[ENCODER_RR] = WheelSpeedControl_Limit(
        rr_x10, -WHEEL_TARGET_X10_LIMIT, WHEEL_TARGET_X10_LIMIT);

    for (i = 0U; i < ENCODER_NUM; i++) {
        int32_t magnitude;

        g_wheel_target_x10[i] = target[i];

        /* 主目标取绝对值最大且保留符号的那个：
         * 前进/后退等于整车目标，横移/旋转时也不会被正负相抵成 0 */
        magnitude = (target[i] >= 0L) ? target[i] : -target[i];
        if (magnitude > main_magnitude) {
            main_magnitude = magnitude;
            main_x10 = target[i];
        }
    }
    wheel_target_main_x10 = main_x10;
}

static void WheelSpeedControl_ClearState(void)
{
    uint8_t i;

    for (i = 0U; i < ENCODER_NUM; i++) {
        wheel_output_pwm[i] = 0L;
        wheel_error_prev_x10[i] = 0L;
        wheel_ff_prev[i] = 0L;
        wheel_target_prev_x10[i] = 0L;
        g_wheel_control_pwm[i] = 0;
    }
}

void WheelSpeedControl_Init(void)
{
    wheel_control_running = 0U;
    wheel_target_main_x10 = 0L;
    WheelSpeedControl_WriteTargets(0L, 0L, 0L, 0L);
    WheelSpeedControl_ClearState();
    Move_Stop();
}

void WheelSpeedControl_Start(int32_t target_speed_x10)
{
    WheelSpeedControl_StartVector(target_speed_x10, target_speed_x10,
                                  target_speed_x10, target_speed_x10);
}

void WheelSpeedControl_StartVector(int32_t lf_x10, int32_t rf_x10,
                                   int32_t lr_x10, int32_t rr_x10)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    WheelSpeedControl_ClearState();
    WheelSpeedControl_WriteTargets(lf_x10, rf_x10, lr_x10, rr_x10);
    wheel_control_running = 1U;
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

void WheelSpeedControl_SetTargets(int32_t lf_x10, int32_t rf_x10,
                                  int32_t lr_x10, int32_t rr_x10)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    WheelSpeedControl_WriteTargets(lf_x10, rf_x10, lr_x10, rr_x10);
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

void WheelSpeedControl_Stop(void)
{
    /*
     * 只调 Move_Stop()：它会顺带调用 WheelSpeedControl_Abort() 关掉闭环，
     * 再断开四路 PWM。反过来这里不能再清一遍，否则与
     * Move_Stop() -> Abort() 的调用链重复（而且很容易写成递归）。
     */
    Move_Stop();
}

void WheelSpeedControl_Abort(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    wheel_control_running = 0U;
    wheel_target_main_x10 = 0L;
    WheelSpeedControl_WriteTargets(0L, 0L, 0L, 0L);
    WheelSpeedControl_ClearState();
    if ((primask & 0x01U) == 0U) {
        __enable_irq();
    }
}

uint8_t WheelSpeedControl_IsRunning(void)
{
    return wheel_control_running;
}

int32_t WheelSpeedControl_GetTargetX10(void)
{
    return wheel_target_main_x10;
}

int32_t WheelSpeedControl_GetWheelTargetX10(Encoder_ID id)
{
    if (id >= ENCODER_NUM) {
        return 0L;
    }
    return g_wheel_target_x10[id];
}

/**
 * @brief  10ms 闭环：增量式 PI + 前馈 + 静摩擦补偿
 * @note   四个轮子各自独立算，但结构完全一致，因此不会人为引入左右差异。
 */
void WheelSpeedControl_Update10ms(void)
{
    uint8_t i;

    if (wheel_control_running == 0U) {
        return;
    }

    for (i = 0U; i < ENCODER_NUM; i++) {
        int32_t target_x10 = WheelSpeedControl_EffectiveTarget(i);
        int32_t speed_x10 = g_wheel_speed_x10[i];
        int32_t feedforward = WheelSpeedControl_Feedforward(target_x10);
        int32_t delta_target = target_x10 - wheel_target_prev_x10[i];
        int32_t error;
        int32_t output;

        /* 目标突变/换向：无扰切换到"纯前馈"，清掉上一工况的误差历史 */
        if ((delta_target > WHEEL_TARGET_JUMP_X10)
            || (delta_target < -WHEEL_TARGET_JUMP_X10)) {
            wheel_output_pwm[i] = feedforward;
            wheel_error_prev_x10[i] = target_x10 - speed_x10;
            wheel_ff_prev[i] = feedforward;
        }
        wheel_target_prev_x10[i] = target_x10;

        error = target_x10 - speed_x10;

        if (target_x10 == 0L) {
            /*
             * 目标为 0：只做阻尼刹车，不再往前推，也不会反向倒车。
             * 原来这种工况输出被前馈/积分顶着，车会"溜"一段才停。
             */
            if ((speed_x10 <= WHEEL_ZERO_SPEED_X10)
                && (speed_x10 >= -WHEEL_ZERO_SPEED_X10)) {
                output = 0L;
            } else {
                output = -(speed_x10 * WHEEL_BRAKE_GAIN_X10) / 10L;
                output = WheelSpeedControl_Limit(output,
                                                 -WHEEL_BRAKE_PWM_LIMIT,
                                                 WHEEL_BRAKE_PWM_LIMIT);
            }
        } else {
            /* 增量式（速度式）PI：输出只累加本周期的变化量，天然抗积分饱和 */
            output = wheel_output_pwm[i]
                   + (WHEEL_PI_KP_X10 * (error - wheel_error_prev_x10[i])) / 10L
                   + (WHEEL_PI_KI_X1000 * error) / 1000L
                   + (feedforward - wheel_ff_prev[i]);
            output = WheelSpeedControl_Limit(output,
                                             -WHEEL_CONTROL_PWM_LIMIT,
                                             WHEEL_CONTROL_PWM_LIMIT);
        }

        wheel_output_pwm[i] = output;
        wheel_error_prev_x10[i] = error;
        wheel_ff_prev[i] = feedforward;
        g_wheel_control_pwm[i] = (int16_t)output;
    }

    Move_SetWheelSpeeds(g_wheel_control_pwm[ENCODER_LF],
                        g_wheel_control_pwm[ENCODER_RF],
                        g_wheel_control_pwm[ENCODER_LR],
                        g_wheel_control_pwm[ENCODER_RR]);
}
