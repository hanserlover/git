/**
 * @file    motion_test.h
 * @brief   四向运动测试界面：按键给出前进/后退/左移/右移指令，OLED 显示状态
 *
 * 全部非阻塞：
 *   MotionTest_ProcessKey()  每 10ms 用 Key_Scan() 的结果调用（只做边沿/长按判定）
 *   MotionTest_ProcessTick() 主循环调用，内部按 10ms 节拍限速刷新 OLED
 */
#ifndef __MOTION_TEST_H
#define __MOTION_TEST_H

#include "main.h"

void MotionTest_Init(void);
void MotionTest_ProcessKey(uint8_t key);
void MotionTest_ProcessTick(uint32_t tick_10ms);

#endif
