/**
 * @file    imu601.h
 * @brief   IMU601 串口通信（头文件）—— ★只保留发送指令，不再解析角度★
 *
 * ============================================================================
 *  ★本文件职责（已精简）★
 *    只负责通过 USART3 向 IMU601 发送控制指令：
 *        1) 软复位指令      IMU_SoftReset_Send()
 *        2) yaw 零点校准指令 IMU_YawCalib_Send()
 *        3) 自定义 yaw 目标  IMU_YawSet_Send()
 *
 *  ★已删除的内容★
 *    原驱动里的姿态帧解析（AA 55 ... 0x60 0x01 0x06 帧）、
 *    Attitude_t 数据结构、IMU601_GetAttitude()、IMU601_IsValid()
 *    等"角度测定"相关代码已全部移除。
 *    原因：本工程业务不再做姿态闭环/角度纠偏，不需要读角度；
 *          而且原来"等 IMU 有效帧才启动"的逻辑正是上电不启动的原因。
 *
 *  ★串口接线★
 *    USART3：TX = PC10，RX = PC11，115200 8N1（部分重映射）
 *    由 CubeMX 的 MX_USART3_UART_Init() 配置，底层 usart.c 未做改动。
 * ============================================================================
 */
#ifndef __IMU601_H
#define __IMU601_H

#include "main.h"
#include "usart.h"      /* 提供 huart3 句柄 */

/* ============================= 串口句柄 ============================= */
#define IMU601_UART_HANDLE        huart3    /* USART3：PC10=TX, PC11=RX */

/* ============================= 指令发送接口 ============================= */

/**
 * @brief  发送 IMU 软复位指令
 *         指令内容：AA 55 60 12 00 72
 * @note   非阻塞调用（只把字节丢进 DR 寄存器，不等对端应答），
 *         发送超时由 HAL 内部保证，正常是一瞬完成。
 */
void IMU_SoftReset_Send(void);

/**
 * @brief  发送 IMU yaw 零点校准指令（内部 yaw 目标为默认值）
 *         指令内容：AA 55 60 14 04 85 0B B5 43 00
 * @note   ★这条指令就是"把当前姿态置为内部 yaw 零点"★。
 *         默认 4 字节负载沿用模块出厂/手册给定的值；
 *         若需要指定不同的内部 yaw 目标，用 IMU_YawSet_Send()。
 */
void IMU_YawCalib_Send(void);

/**
 * @brief  发送 yaw 零点校准指令，并指定内部 yaw 目标值
 * @param  yaw_deg 目标 yaw 角（度），按 float 小端写入 4 字节负载
 * @note   帧格式与 IMU_YawCalib_Send() 完全一致，只是负载换成 yaw_deg：
 *             AA 55 60 14 04 [yaw 4字节小端] [校验和]
 *         校验和 = 字节 2~8 的累加和取低 8 位。
 *         IMU_YawCalib_Send() 等价于发送手册默认负载，
 *         若要"修改内部 yaw 目标数值"就用本函数。
 */
void IMU_YawSet_Send(float yaw_deg);

/**
 * @brief  IMU 通信层初始化（上电调用一次）
 * @note   只清内部状态，不发送任何指令、不含任何延时。
 *         具体"先软复位、再 yaw 校准"的时序由业务层
 *         （calib_1m.c 的上电时序状态机）非阻塞推进。
 */
void IMU601_Init(void);

/**
 * @brief  丢弃 IMU 主动上报的数据（不解析、不使用）
 * @note   本工程不需要任何角度数据。但模块上电后会持续通过串口上报，
 *         把接收寄存器读空可以避免接收溢出（ORE）标志长期挂起，
 *         保持串口干净、不影响后续发送。
 *         不解析内容，因此不属于"角度测定"。
 */
void IMU601_Process(void);

/**
 * @brief  取出 USART3 收到的原始字节（不做任何解析）
 * @param  buffer  输出缓冲
 * @param  max_len 最多取多少字节
 * @return 实际取到的字节数（0 表示当前没有数据）
 * @note   非阻塞：有多少读多少，读 DR 同时清 RXNE/ORE。
 *         正常业务用 IMU601_Process() 把数据读空即可；
 *         这个接口是给"协议嗅探"用的——把原始字节转到调试串口，
 *         抓一段 hex 出来就能反推模块的上报帧格式。
 */
uint16_t IMU601_ReadRaw(uint8_t *buffer, uint16_t max_len);

#endif /* __IMU601_H */
