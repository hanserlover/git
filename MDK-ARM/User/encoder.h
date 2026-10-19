#ifndef __ENCODER_H
#define __ENCODER_H

#include "tim.h"

/* 硬件编码器定时器映射 */
#define ENCODER_LF_TIM     (&htim4)   /* 左前 PB6/PB7 */
#define ENCODER_RF_TIM     (&htim8)   /* 右前 PC6/PC7 */

/* 实测方向修正：统一规定车轮物理前进时编码器计数为正。 */
#define ENCODER_SIGN_LF    (-1L)
#define ENCODER_SIGN_RF    ( 1L)
#define ENCODER_SIGN_LR    ( 1L)
#define ENCODER_SIGN_RR    (-1L)

/* 软件编码器引脚 (EXTI双边沿) */
#define ENC_SW_LR_A_PORT   GPIOB      /* 左后 A */
#define ENC_SW_LR_A_PIN    GPIO_PIN_8
#define ENC_SW_LR_B_PORT   GPIOB      /* 左后 B */
#define ENC_SW_LR_B_PIN    GPIO_PIN_9

#define ENC_SW_RR_A_PORT   GPIOB      /* 右后 A */
#define ENC_SW_RR_A_PIN    GPIO_PIN_0
#define ENC_SW_RR_B_PORT   GPIOB      /* 右后 B */
#define ENC_SW_RR_B_PIN    GPIO_PIN_1

typedef enum {
    ENCODER_LF = 0,        /* 左前 硬件 */
    ENCODER_RF,            /* 右前 硬件 */
    ENCODER_LR,            /* 左后 软件 */
    ENCODER_RR,            /* 右后 软件 */
    ENCODER_NUM
} Encoder_ID;

void    Encoder_Init(void);
int32_t Encoder_GetCount(Encoder_ID id);
int32_t Encoder_GetPhysicalCount(Encoder_ID id);
void    Encoder_Clear(Encoder_ID id);

int32_t Encoder_GetAvgCount(void);        
int32_t Encoder_GetForwardCount(void);    
void    Encoder_ClearAll(void);           

#endif
