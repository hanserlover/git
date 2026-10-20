#include "encoder.h"

#define ENC_SW_LR_IDX  0
#define ENC_SW_RR_IDX  1

static volatile int32_t enc_sw_count[2] = {0, 0};
static uint8_t  enc_sw_state[2] = {0, 0};

static const int8_t enc_quad_table[4][4] = {
    { 0,  1, -1,  0},
    {-1,  0,  0,  1},
    { 1,  0,  0, -1},
    { 0, -1,  1,  0}
};

static void Enc_SoftUpdate(uint8_t idx, uint8_t new_state)
{
    enc_sw_count[idx] += enc_quad_table[enc_sw_state[idx]][new_state];
    enc_sw_state[idx] = new_state;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t a, b, s;

    /* 左后软件编码器 PB8(A) / PB9(B) */
    if (GPIO_Pin == ENC_SW_LR_A_PIN || GPIO_Pin == ENC_SW_LR_B_PIN) {
        a = (HAL_GPIO_ReadPin(ENC_SW_LR_A_PORT, ENC_SW_LR_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
        b = (HAL_GPIO_ReadPin(ENC_SW_LR_B_PORT, ENC_SW_LR_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
        s = (uint8_t)((a << 1) | b);
        Enc_SoftUpdate(ENC_SW_LR_IDX, s);
    }
    /* 右后软件编码器 PB0(A) / PB1(B) */
    else if (GPIO_Pin == ENC_SW_RR_A_PIN || GPIO_Pin == ENC_SW_RR_B_PIN) {
        a = (HAL_GPIO_ReadPin(ENC_SW_RR_A_PORT, ENC_SW_RR_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
        b = (HAL_GPIO_ReadPin(ENC_SW_RR_B_PORT, ENC_SW_RR_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
        s = (uint8_t)((a << 1) | b);
        Enc_SoftUpdate(ENC_SW_RR_IDX, s);
    }
}

void Encoder_Init(void)
{
    uint8_t a, b;

    HAL_TIM_Encoder_Start(ENCODER_LF_TIM, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(ENCODER_LF_TIM, 0);

    HAL_TIM_Encoder_Start(ENCODER_RF_TIM, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(ENCODER_RF_TIM, 0);

    enc_sw_count[ENC_SW_LR_IDX] = 0;
    enc_sw_count[ENC_SW_RR_IDX] = 0;

    a = (HAL_GPIO_ReadPin(ENC_SW_LR_A_PORT, ENC_SW_LR_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
    b = (HAL_GPIO_ReadPin(ENC_SW_LR_B_PORT, ENC_SW_LR_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    enc_sw_state[ENC_SW_LR_IDX] = (uint8_t)((a << 1) | b);

    a = (HAL_GPIO_ReadPin(ENC_SW_RR_A_PORT, ENC_SW_RR_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
    b = (HAL_GPIO_ReadPin(ENC_SW_RR_B_PORT, ENC_SW_RR_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    enc_sw_state[ENC_SW_RR_IDX] = (uint8_t)((a << 1) | b);
}

int32_t Encoder_GetCount(Encoder_ID id)
{
    switch (id) {
        case ENCODER_LF: return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(ENCODER_LF_TIM);
        case ENCODER_RF: return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(ENCODER_RF_TIM);
        case ENCODER_LR: return enc_sw_count[ENC_SW_LR_IDX];
        case ENCODER_RR: return enc_sw_count[ENC_SW_RR_IDX];
        default:         return 0;
    }
}

int32_t Encoder_GetPhysicalCount(Encoder_ID id)
{
    int32_t raw = Encoder_GetCount(id);

    switch (id) {
        case ENCODER_LF: return raw * ENCODER_SIGN_LF;
        case ENCODER_RF: return raw * ENCODER_SIGN_RF;
        case ENCODER_LR: return raw * ENCODER_SIGN_LR;
        case ENCODER_RR: return raw * ENCODER_SIGN_RR;
        default:         return 0;
    }
}

void Encoder_Clear(Encoder_ID id)
{
    switch (id) {
        case ENCODER_LF: __HAL_TIM_SET_COUNTER(ENCODER_LF_TIM, 0); break;
        case ENCODER_RF: __HAL_TIM_SET_COUNTER(ENCODER_RF_TIM, 0); break;
        case ENCODER_LR: enc_sw_count[ENC_SW_LR_IDX] = 0;         break;
        case ENCODER_RR: enc_sw_count[ENC_SW_RR_IDX] = 0;         break;
        default:         break;
    }
}

int32_t Encoder_GetAvgCount(void)
{
    int32_t sum = Encoder_GetCount(ENCODER_LF) + Encoder_GetCount(ENCODER_RF) + 
                  Encoder_GetCount(ENCODER_LR) + Encoder_GetCount(ENCODER_RR);
    return sum / 4;
}

int32_t Encoder_GetForwardCount(void)
{
    int32_t sum = Encoder_GetCount(ENCODER_LF) + Encoder_GetCount(ENCODER_RF);
    return sum / 2;
}

void Encoder_ClearAll(void)
{
    Encoder_Clear(ENCODER_LF);
    Encoder_Clear(ENCODER_RF);
    Encoder_Clear(ENCODER_LR);
    Encoder_Clear(ENCODER_RR);
}
