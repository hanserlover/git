/**
 * @file    motor.c
 * @brief   4 路麦轮电机驱动（TB6612 两片）实现
 *
 * ★本次修改★
 *   motor_tab 表按 motor.h 的新引脚重建：左前↔左后、右前↔右后 对调。
 *   表项顺序必须与 Motor_ID 枚举一致（LF, LR, RF, RR）。
 */
#include "motor.h"

/* 单电机控制资源 */
typedef struct {
    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;
    TIM_HandleTypeDef *tim;
    uint32_t           ch;
} Motor_Handle;

/* ★对调★ 索引 0=LF 用原左后引脚，1=LR 用原左前引脚，
 *          2=RF 用原右后引脚，3=RR 用原右前引脚 */
static const Motor_Handle motor_tab[MOTOR_NUM] = {
    {MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, MOTOR_LF_IN2_PORT, MOTOR_LF_IN2_PIN,
     MOTOR_LF_PWM_TIM, MOTOR_LF_PWM_CH},   /* 左前：PC8/PC9 + PA10 */
    {MOTOR_LR_IN1_PORT, MOTOR_LR_IN1_PIN, MOTOR_LR_IN2_PORT, MOTOR_LR_IN2_PIN,
     MOTOR_LR_PWM_TIM, MOTOR_LR_PWM_CH},   /* 左后：PC2/PC3 + PA8  */
    {MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, MOTOR_RF_IN2_PORT, MOTOR_RF_IN2_PIN,
     MOTOR_RF_PWM_TIM, MOTOR_RF_PWM_CH},   /* 右前：PC12/PB5 + PA11 */
    {MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN, MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN,
     MOTOR_RR_PWM_TIM, MOTOR_RR_PWM_CH}    /* 右后：PC4/PC5 + PB4   */
};

/**
 * @brief  初始化：使能两片 TB6612，启动 4 路 PWM，全部停止
 */
void Motor_Init(void)
{
    uint8_t i;

    HAL_GPIO_WritePin(MOTOR_TBL_STBY_PORT, MOTOR_TBL_STBY_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_TBR_STBY_PORT, MOTOR_TBR_STBY_PIN, GPIO_PIN_SET);

    for (i = 0; i < MOTOR_NUM; i++) {
        HAL_TIM_PWM_Start(motor_tab[i].tim, motor_tab[i].ch);
        Motor_SetDir((Motor_ID)i, MOTOR_STOP);
        __HAL_TIM_SET_COMPARE(motor_tab[i].tim, motor_tab[i].ch, 0);
    }
}

/**
 * @brief  设置方向（TB6612：IN1/IN2 = 1/0 正转，0/1 反转，1/1 刹车，0/0 停止）
 */
void Motor_SetDir(Motor_ID id, Motor_Dir dir)
{
    GPIO_PinState a;
    GPIO_PinState b;

    if (id >= MOTOR_NUM) {
        return;
    }

    switch (dir) {
        case MOTOR_FORWARD: a = GPIO_PIN_SET;   b = GPIO_PIN_RESET; break;
        case MOTOR_REVERSE: a = GPIO_PIN_RESET; b = GPIO_PIN_SET;   break;
        case MOTOR_BRAKE:   a = GPIO_PIN_SET;   b = GPIO_PIN_SET;   break;
        case MOTOR_STOP:
        default:            a = GPIO_PIN_RESET; b = GPIO_PIN_RESET; break;
    }

    HAL_GPIO_WritePin(motor_tab[id].in1_port, motor_tab[id].in1_pin, a);
    HAL_GPIO_WritePin(motor_tab[id].in2_port, motor_tab[id].in2_pin, b);
}

/**
 * @brief  设置 PWM 占空比 0~MOTOR_PWM_MAX
 */
void Motor_SetPWM(Motor_ID id, uint16_t duty)
{
    if (id >= MOTOR_NUM) {
        return;
    }
    if (duty > MOTOR_PWM_MAX) {
        duty = MOTOR_PWM_MAX;
    }
    __HAL_TIM_SET_COMPARE(motor_tab[id].tim, motor_tab[id].ch, duty);
}

/**
 * @brief  带符号速度：正=正转，负=反转
 */
void Motor_SetSpeed(Motor_ID id, int16_t speed)
{
    uint16_t duty;

    if (speed >= 0) {
        Motor_SetDir(id, MOTOR_FORWARD);
        duty = (uint16_t)speed;
    } else {
        Motor_SetDir(id, MOTOR_REVERSE);
        duty = (uint16_t)(-speed);
    }
    Motor_SetPWM(id, duty);
}

void Motor_Stop(Motor_ID id)
{
    Motor_SetDir(id, MOTOR_STOP);
    Motor_SetPWM(id, 0);
}

void Motor_Brake(Motor_ID id)
{
    Motor_SetDir(id, MOTOR_BRAKE);
    Motor_SetPWM(id, 0);
}

/* ===================== 单轮正转 / 反转 ===================== */

void Motor_LF_Fwd(uint16_t pwm) { Motor_SetDir(MOTOR_LF, MOTOR_FORWARD); Motor_SetPWM(MOTOR_LF, pwm); }
void Motor_LF_Rev(uint16_t pwm) { Motor_SetDir(MOTOR_LF, MOTOR_REVERSE); Motor_SetPWM(MOTOR_LF, pwm); }

void Motor_LR_Fwd(uint16_t pwm) { Motor_SetDir(MOTOR_LR, MOTOR_FORWARD); Motor_SetPWM(MOTOR_LR, pwm); }
void Motor_LR_Rev(uint16_t pwm) { Motor_SetDir(MOTOR_LR, MOTOR_REVERSE); Motor_SetPWM(MOTOR_LR, pwm); }

void Motor_RF_Fwd(uint16_t pwm) { Motor_SetDir(MOTOR_RF, MOTOR_FORWARD); Motor_SetPWM(MOTOR_RF, pwm); }
void Motor_RF_Rev(uint16_t pwm) { Motor_SetDir(MOTOR_RF, MOTOR_REVERSE); Motor_SetPWM(MOTOR_RF, pwm); }

void Motor_RR_Fwd(uint16_t pwm) { Motor_SetDir(MOTOR_RR, MOTOR_FORWARD); Motor_SetPWM(MOTOR_RR, pwm); }
void Motor_RR_Rev(uint16_t pwm) { Motor_SetDir(MOTOR_RR, MOTOR_REVERSE); Motor_SetPWM(MOTOR_RR, pwm); }
