/**
 * @file    motion_test.h
 * @brief   单按键直行测试：中间按键启动，走满 1.00m 自动停车
 *
 * 全部非阻塞：
 *   MotionTest_ProcessKey()  每 10ms 用 Key_Scan() 的结果调用（只认按下边沿）
 *   MotionTest_ProcessTick() 主循环调用，内部按 10ms 节拍限速刷新 OLED
 */
#ifndef __MOTION_TEST_H
#define __MOTION_TEST_H

#include "main.h"

void MotionTest_Init(void);
void MotionTest_ProcessKey(uint8_t key);
void MotionTest_ProcessTick(uint32_t tick_10ms);

#endif
