/**
 * @file    imu601.c
 * @brief   IMU601 串口通信（实现）—— ★只发指令，不解析角度★
 *
 * ============================================================================
 *  ★指令帧格式（与手册一致）★
 *
 *    软复位：   AA 55 60 12 00 72
 *              └帧头┘ 类型 长度 负载 校验
 *
 *    yaw 校准： AA 55 60 14 04 85 0B B5 43 00
 *              └帧头┘ 类型 长度 └─ yaw 4字节 ─┘ 校验
 *
 *    校验和规则：从第 3 个字节(下标 2)开始累加，一直加到校验和前一字节，
 *                取累加和低 8 位。
 *      软复位： 0x60+0x12+0x00            = 0x72
 *      yaw校准：0x60+0x14+0x04+0x85+0x0B+0xB5+0x43 = 0x200 -> 低8位 0x00
 * ============================================================================
 */
#include "imu601.h"

/* ##########################################################################
 * #                                                                        #
 * #                      ★★★  参 数 区  ★★★                              #
 * #                IMU 指令发送相关可调参数集中在这里                        #
 * #                                                                        #
 * ########################################################################## */

/* ------------------------------------------------------------------------
 * 参数 1：单次发送超时（毫秒）
 * ------------------------------------------------------------------------
 * 只是 HAL 内部发送完成的超时上限，不是"等待对方应答"。
 * 115200 波特率下 10 字节约 0.9ms，给 20ms 足够宽松。
 * 正常情况下一瞬返回，不会阻塞业务节拍。
 */
#define IMU_TX_TIMEOUT_MS         20U

/* ------------------------------------------------------------------------
 * 参数 2：yaw 零点校准指令的默认 4 字节负载
 * ------------------------------------------------------------------------
 * 手册给定值：85 0B B5 43（小端 float）。
 * 若现场需要不同的内部 yaw 目标，直接改这 4 个字节，
 * 或者改用 IMU_YawSet_Send(度数) 由程序自动打包 + 算校验和。
 */
#define IMU_YAW_CALIB_B0          0x85U
#define IMU_YAW_CALIB_B1          0x0BU
#define IMU_YAW_CALIB_B2          0xB5U
#define IMU_YAW_CALIB_B3          0x43U

/* ##########################################################################
 * #                          参数区结束                                     #
 * ########################################################################## */

/* ============================= 内部缓存 ============================= */

/* 发送缓冲：10 字节足够放下两条指令（软复位 6 字节 / yaw 校准 10 字节）。
 * 用静态数组而不是栈变量，避免反复进出函数时压栈开销。 */
static uint8_t s_tx_buf[10];

/* ============================= 内部工具 ============================= */

/**
 * @brief  按校验和规则填充并发送一帧
 * @param  len 整帧长度（含帧头与校验和字节）
 * @note   校验和 = s_tx_buf[2] .. s_tx_buf[len-2] 累加和低 8 位，
 *         写入 s_tx_buf[len-1]。
 */
static void IMU_SendFrame(uint8_t len)
{
    uint8_t i;
    uint8_t sum = 0U;

    if (len < 4U) {
        return;
    }

    for (i = 2U; i < (uint8_t)(len - 1U); i++) {
        sum = (uint8_t)(sum + s_tx_buf[i]);
    }
    s_tx_buf[len - 1U] = sum;

    /* 非阻塞式发送：HAL 内部按字节轮询 DR 空标志，超时由宏控制。
     * 正常 10 字节 @115200 约 0.9ms，远小于 5ms 业务周期。 */
    (void)HAL_UART_Transmit(&IMU601_UART_HANDLE,
                            s_tx_buf,
                            (uint16_t)len,
                            (uint32_t)IMU_TX_TIMEOUT_MS);
}

/* ============================= 对外接口 ============================= */

/**
 * @brief  初始化 IMU 通信层
 * @note   只清状态，不发指令、不延时。
 *         上电先软复位、再 yaw 校准的时序由 calib_1m.c 非阻塞推进。
 */
void IMU601_Init(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)sizeof(s_tx_buf); i++) {
        s_tx_buf[i] = 0U;
    }
}

/**
 * @brief  发送软复位指令：AA 55 60 12 00 72
 */
void IMU_SoftReset_Send(void)
{
    s_tx_buf[0] = 0xAAU;    /* 帧头 1 */
    s_tx_buf[1] = 0x55U;    /* 帧头 2 */
    s_tx_buf[2] = 0x60U;    /* 类型 */
    s_tx_buf[3] = 0x12U;    /* 长度标识 */
    s_tx_buf[4] = 0x00U;    /* 负载 */
    s_tx_buf[5] = 0x00U;    /* 校验和占位，由 IMU_SendFrame 填 */

    IMU_SendFrame(6U);      /* 校验和 = 0x60+0x12+0x00 = 0x72 */
}

/**
 * @brief  发送 yaw 零点校准指令（手册默认负载）
 *         指令内容：AA 55 60 14 04 85 0B B5 43 00
 */
void IMU_YawCalib_Send(void)
{
    s_tx_buf[0] = 0xAAU;
    s_tx_buf[1] = 0x55U;
    s_tx_buf[2] = 0x60U;
    s_tx_buf[3] = 0x14U;
    s_tx_buf[4] = 0x04U;                        /* 负载 4 字节 */
    s_tx_buf[5] = IMU_YAW_CALIB_B0;
    s_tx_buf[6] = IMU_YAW_CALIB_B1;
    s_tx_buf[7] = IMU_YAW_CALIB_B2;
    s_tx_buf[8] = IMU_YAW_CALIB_B3;
    s_tx_buf[9] = 0x00U;                        /* 校验和占位 */

    IMU_SendFrame(10U);     /* 校验和低 8 位 = 0x00 */
}

/**
 * @brief  发送 yaw 零点校准指令，并指定内部 yaw 目标值
 * @param  yaw_deg 目标 yaw（度），按 float 小端打包进 4 字节负载
 */
void IMU_YawSet_Send(float yaw_deg)
{
    union {
        float    f;
        uint32_t u;
    } conv;
    uint8_t i;

    conv.f = yaw_deg;

    s_tx_buf[0] = 0xAAU;
    s_tx_buf[1] = 0x55U;
    s_tx_buf[2] = 0x60U;
    s_tx_buf[3] = 0x14U;
    s_tx_buf[4] = 0x04U;

    /* float 按小端写入：低字节在前 */
    for (i = 0U; i < 4U; i++) {
        s_tx_buf[5U + i] = (uint8_t)((conv.u >> (8U * i)) & 0xFFU);
    }
    s_tx_buf[9] = 0x00U;    /* 校验和占位 */

    IMU_SendFrame(10U);
}

/**
 * @brief  丢弃 IMU 上报的数据，不解析
 * @note   只把接收数据寄存器读空，避免 ORE 溢出标志长期挂起。
 */
void IMU601_Process(void)
{
    volatile uint8_t dummy;

    while (__HAL_UART_GET_FLAG(&IMU601_UART_HANDLE, UART_FLAG_RXNE) != RESET) {
        dummy = (uint8_t)(IMU601_UART_HANDLE.Instance->DR & 0x00FFU);
        (void)dummy;
    }
}
