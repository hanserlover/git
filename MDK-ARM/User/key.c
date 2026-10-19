#include "key.h"

#define KEY_DEBOUNCE_CNT 2 

static uint8_t key_last_state = KEY_NONE;
static uint8_t key_cnt = 0;

void Key_Init(void) {
    // 已在 CubeMX 中配置上拉输入
}

uint8_t Key_Scan(void) {
    uint8_t current_state = KEY_NONE;
    
    // 根据最新引脚表读取
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) current_state = KEY_UP;
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) current_state = KEY_LEFT;
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) current_state = KEY_DOWN;
    else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET) current_state = KEY_RIGHT;
    else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) current_state = KEY_CENTER;

    // 状态机消抖
    if (current_state != key_last_state) {
        key_cnt = 0;
        key_last_state = current_state;
    } else {
        if (key_cnt < KEY_DEBOUNCE_CNT) {
            key_cnt++;
        }
    }
    
    // 如果消抖完成，持续返回当前状态；否则返回 KEY_NONE
    if (key_cnt >= KEY_DEBOUNCE_CNT) {
        return current_state;
    }
    return KEY_NONE; 
}
