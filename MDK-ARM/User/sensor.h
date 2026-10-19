#ifndef __SENSOR_H
#define __SENSOR_H

#include "main.h"

/* 触发有效电平：NPN 常开，触发时输出低电平 */
#define SENSOR_ACTIVE_LEVEL GPIO_PIN_RESET

/* 传感器引脚定义 */
#define SENSOR_LF_PORT     GPIOA     /* 左前 避障 */
#define SENSOR_LF_PIN      GPIO_PIN_2

#define SENSOR_RF_PORT     GPIOA     /* 右前 避障 */
#define SENSOR_RF_PIN      GPIO_PIN_3

#define SENSOR_LD_PORT     GPIOB     /* 左侧向下 循迹/边界 */
#define SENSOR_LD_PIN      GPIO_PIN_12

#define SENSOR_LS_PORT     GPIOA     /* 左侧左斜下 避障 */
#define SENSOR_LS_PIN      GPIO_PIN_12

typedef enum {
    SENSOR_LF = 0,        
    SENSOR_RF,            
    SENSOR_LD,            
    SENSOR_LS,            
    SENSOR_NUM
} Sensor_ID;

void    Sensor_Init(void);
uint8_t Sensor_GetRaw(Sensor_ID id);
uint8_t Sensor_GetState(Sensor_ID id);

#endif
