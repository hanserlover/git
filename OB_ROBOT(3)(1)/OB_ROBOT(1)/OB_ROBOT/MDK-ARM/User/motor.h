#ifndef __MOTOR_H
#define __MOTOR_H

#include "tim.h"

/* ===================== 左前 LF ===================== */
#define MOTOR_LF_IN1_PORT   GPIOC
#define MOTOR_LF_IN1_PIN    GPIO_PIN_8
#define MOTOR_LF_IN2_PORT   GPIOC
#define MOTOR_LF_IN2_PIN    GPIO_PIN_9
#define MOTOR_LF_PWM_TIM    (&htim1)
#define MOTOR_LF_PWM_CH     TIM_CHANNEL_3          /* PA10 */

/* ===================== 左后 LR ===================== */
#define MOTOR_LR_IN1_PORT   GPIOC
#define MOTOR_LR_IN1_PIN    GPIO_PIN_2
#define MOTOR_LR_IN2_PORT   GPIOC
#define MOTOR_LR_IN2_PIN    GPIO_PIN_3
#define MOTOR_LR_PWM_TIM    (&htim1)
#define MOTOR_LR_PWM_CH     TIM_CHANNEL_1          /* PA8 */

/* ===================== 右前 RF ===================== */
#define MOTOR_RF_IN1_PORT   GPIOC
#define MOTOR_RF_IN1_PIN    GPIO_PIN_4
#define MOTOR_RF_IN2_PORT   GPIOC
#define MOTOR_RF_IN2_PIN    GPIO_PIN_5
#define MOTOR_RF_PWM_TIM    (&htim3)
#define MOTOR_RF_PWM_CH     TIM_CHANNEL_1          /* PB4 */

/* ===================== 右后 RR ===================== */
#define MOTOR_RR_IN1_PORT   GPIOC
#define MOTOR_RR_IN1_PIN    GPIO_PIN_12
#define MOTOR_RR_IN2_PORT   GPIOB
#define MOTOR_RR_IN2_PIN    GPIO_PIN_5
#define MOTOR_RR_PWM_TIM    (&htim1)
#define MOTOR_RR_PWM_CH     TIM_CHANNEL_4          /* PA11 */

/* ===================== 待机脚 ===================== */
#define MOTOR_TBL_STBY_PORT GPIOC                  /* 左片 TB6612 */
#define MOTOR_TBL_STBY_PIN  GPIO_PIN_0
#define MOTOR_TBR_STBY_PORT GPIOC                  /* 右片 TB6612 */
#define MOTOR_TBR_STBY_PIN  GPIO_PIN_1

#define MOTOR_PWM_MAX       3599

typedef enum {
    MOTOR_LF = 0,          /* 左前 */
    MOTOR_LR,              /* 左后 */
    MOTOR_RF,              /* 右前 */
    MOTOR_RR,              /* 右后 */
    MOTOR_NUM
} Motor_ID;

typedef enum {
    MOTOR_STOP = 0,        
    MOTOR_FORWARD,         
    MOTOR_REVERSE,         
    MOTOR_BRAKE            
} Motor_Dir;

void    Motor_Init(void);
void    Motor_SetDir(Motor_ID id, Motor_Dir dir);
void    Motor_SetPWM(Motor_ID id, uint16_t duty);
void    Motor_SetSpeed(Motor_ID id, int16_t speed);
void    Motor_Stop(Motor_ID id);
void    Motor_Brake(Motor_ID id);

#endif
