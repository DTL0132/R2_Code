/**
 * @file    vofa_uart.h
 * @brief   VOFA+ 串口发送接口（极简版）
 *
 * @details 发送数据格式：
 *          [N 个 float（32 位，小端）] + [+Inf 帧尾 0x7F800000]
 *          VOFA+ 端配置为「小端 float + +Inf 结尾」即可直接画波形。
 */
#ifndef VOFA_UART_H
#define VOFA_UART_H

#include "main.h"

/**
 * @brief  单帧允许发送的最大 float 个数
 *
 * 决定内部打包缓冲区大小：
 *   缓冲大小 = VOFA_TX_MAX_FLOATS * 4 字节 + 4 字节帧尾
 *
 * 可根据实际波形通道数 / 发送频率酌情调节。
 */
#ifndef VOFA_TX_MAX_FLOATS
#define VOFA_TX_MAX_FLOATS (64u)
#endif

/**
 * @brief  阻塞发送：打包 N 个 float + 帧尾后一次性发送
 *
 * @param  huart  串口句柄（如 &huart3）
 * @param  data   连续 float 数组首地址
 * @param  n      浮点个数（> VOFA_TX_MAX_FLOATS 将被截断）
 *
 * @note   适合在主循环或低频调试场景使用，简单可靠但会占用 CPU 等待发送完成。
 */
void VOFA_SendFloats_Blocking(UART_HandleTypeDef *huart,
                              const float *data,
                              uint16_t n);

/**
 * @brief  使用 DMA 发送：打包 N 个 float + 帧尾后启动一次 DMA
 *
 * @param  huart  串口句柄（如 &huart3，需预先使能 UART + DMA）
 * @param  data   连续 float 数组首地址
 * @param  n      浮点个数（> VOFA_TX_MAX_FLOATS 将被截断）
 *
 * @note   - 内部仅做最小化忙检测，不负责 DMA 回调与错误处理。
 *         - 适合在定时中断或调度器中周期性调用，减轻 CPU 发送负担。
 */
void VOFA_SendFloats_DMA(UART_HandleTypeDef *huart,
                         const float *data,
                         uint16_t n);

#endif /* VOFA_UART_H */
