/**
 * @file    sensor.c
 * @brief   6 路 NPN 常开光电传感器底层驱动（实现）
 */
#include "sensor.h"

/* 传感器 GPIO 查找表 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} Sensor_Gpio;

static const Sensor_Gpio sensor_tab[SENSOR_NUM] = {
    {SENSOR_LF_PORT,  SENSOR_LF_PIN},   
    {SENSOR_RF_PORT,  SENSOR_RF_PIN},   
    {SENSOR_LD_PORT,  SENSOR_LD_PIN},   
    {SENSOR_LS_PORT,  SENSOR_LS_PIN}    
};

/**
 * @brief  传感器初始化（GPIO 输入已在 CubeMX 中配置，此处无额外操作）
 */
void Sensor_Init(void)
{
    /* nothing to do */
}

/**
 * @brief  读取传感器原始电平
 * @param  id 传感器编号
 * @retval 1=高电平，0=低电平
 */
uint8_t Sensor_GetRaw(Sensor_ID id)
{
    if (id >= SENSOR_NUM) {
        return 0;
    }
    return (HAL_GPIO_ReadPin(sensor_tab[id].port, sensor_tab[id].pin) == GPIO_PIN_SET) ? 1 : 0;
}

/**
 * @brief  读取传感器触发状态
 * @param  id 传感器编号
 * @retval 1=触发，0=未触发
 */
uint8_t Sensor_GetState(Sensor_ID id)
{
    return (Sensor_GetRaw(id) == SENSOR_ACTIVE_LEVEL);
}

/**
 * @brief  读取全部传感器触发状态
 * @retval 8 位掩码，bit i 对应 Sensor_ID i，1=触发
 */
uint8_t Sensor_GetAll(void)
{
    uint8_t i;
    uint8_t mask = 0;
    for (i = 0; i < SENSOR_NUM; i++) {
        if (Sensor_GetState((Sensor_ID)i)) {
            mask |= (1u << i);
        }
    }
    return mask;
}
