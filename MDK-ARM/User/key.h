#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#define KEY_NONE    0
#define KEY_UP      1
#define KEY_DOWN    2
#define KEY_LEFT    3
#define KEY_RIGHT   4
#define KEY_CENTER  5

void Key_Init(void);
uint8_t Key_Scan(void); // 需在 10ms 周期任务中调用

#endif
