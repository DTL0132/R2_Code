/**
 * @file    vofa_uart.c
 * @brief   VOFA+ 串口发送实现（阻塞版 + DMA 版）
 * @details 发送格式固定为：
 *          [N 个 32 位 float（小端）] + [尾标记 +Inf：0x7F800000]
 */
#include "vofa_uart.h"
#include <string.h>
#include <stdint.h>

/*==================================================================================================
 * 私有常量与缓冲区
 *================================================================================================*/

/**
 * @brief VOFA+ 帧尾标记（IEEE754 单精度 +Inf）
 *
 * 小端发送字节序：00 00 80 7F
 * VOFA+ 通过该特殊值识别一帧数据的结束位置。
 */
static const uint32_t s_vofa_tail = 0x7F800000u;

/* 单个 float 占用字节数（32 位） */
#define VOFA_TAIL_SIZE (4u)
#define VOFA_FLOAT_SIZE (4u)

/* 最多支持发送的浮点字节数：VOFA_TX_MAX_FLOATS 个 float */
#define VOFA_MAX_DATA_BYTES (VOFA_TX_MAX_FLOATS * VOFA_FLOAT_SIZE)

/**
 * @brief 发送打包缓冲区
 *
 * 用于存放：N 个 float 数据 + 4 字节帧尾。
 * 阻塞发送与 DMA 发送共用该缓冲区，因此：
 * - 不适合在多线程/中断重入环境下同时调用两个接口
 * - 若需重入，请改为每通道/每实例独立缓冲区
 */
static uint8_t s_vofa_tx_buf[VOFA_MAX_DATA_BYTES + VOFA_TAIL_SIZE];

/**
 * @brief  阻塞方式发送：一次打包 N 个 float + 帧尾
 *
 * @param  huart  串口句柄（例如 &huart3）
 * @param  data   指向连续 float 数组的指针
 * @param  n      需要发送的浮点个数（超过 VOFA_TX_MAX_FLOATS 会被截断）
 *
 * @note   内部使用 HAL_UART_Transmit，超时时间固定为 100 ms。
 *         不检查返回值，只做简单 (void) 丢弃，可根据需要自行扩展错误处理。
 */
void VOFA_SendFloats_Blocking(UART_HandleTypeDef *huart,
                              const float *data,
                              uint16_t n)
{
    /* 限制最多发送 VOFA_TX_MAX_FLOATS 个点，防止越界 */
    if (n > VOFA_TX_MAX_FLOATS)
        n = VOFA_TX_MAX_FLOATS;

    /* 计算需要拷贝的浮点数据字节数 */
    uint16_t bytes_data = (uint16_t)(n * VOFA_FLOAT_SIZE);

    /* 拷贝 N 个 float 到发送缓冲（内存连续即可） */
    memcpy(s_vofa_tx_buf, data, bytes_data);

    /* 在数据后紧跟写入 4 字节帧尾标记 */
    *(uint32_t *)&s_vofa_tx_buf[bytes_data] = s_vofa_tail;

    /* 一次性阻塞发送：N 个 float + 帧尾 */
    (void)HAL_UART_Transmit(huart,
                            s_vofa_tx_buf,
                            (uint16_t)(bytes_data + VOFA_TAIL_SIZE),
                            100u);
}

/**
 * @brief  使用 DMA 一次性发送：N 个 float + 帧尾
 *
 * @param  huart  串口句柄（需提前配置好对应 UART + DMA）
 * @param  data   指向连续 float 数组的指针
 * @param  n      需要发送的浮点个数（超过 VOFA_TX_MAX_FLOATS 会被截断）
 *
 * @note   - 若 UART 正在忙（包括上一次 DMA 发送未完成），本函数直接返回，不做等待。
 *         - 内部使用全局缓冲区打包，适合「周期性波形上报」这类简单场景。
 *         - HAL 错误码（包括 HAL_BUSY）在此接口中不向上传递，如需精细处理可自行修改。
 */
void VOFA_SendFloats_DMA(UART_HandleTypeDef *huart,  const float *data, uint16_t n)
{
    /* 简单忙检测：避免在 DMA 传输未完成时重入打包缓冲区 */
    if (huart->gState != HAL_UART_STATE_READY)
        return;

    /* 限制最多发送 VOFA_TX_MAX_FLOATS 个点，防止越界 */
    if (n > VOFA_TX_MAX_FLOATS)
        n = VOFA_TX_MAX_FLOATS;

    /* 计算需要拷贝的浮点数据字节数 */
    uint16_t bytes_data = (uint16_t)(n * VOFA_FLOAT_SIZE);

    /* 拷贝 N 个 float 到发送缓冲 */
    memcpy(s_vofa_tx_buf, data, bytes_data);

    /* 在数据后紧跟写入 4 字节帧尾标记 */
    *(uint32_t *)&s_vofa_tx_buf[bytes_data] = s_vofa_tail;

    /* 启动一次 DMA 发送：N 个 float + 帧尾 */
    (void)HAL_UART_Transmit_DMA(huart,
                                s_vofa_tx_buf,
                                (uint16_t)(bytes_data + VOFA_TAIL_SIZE));
}
