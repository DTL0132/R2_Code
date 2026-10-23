#include "main.h"
#include "dma.h"
#include "sbus.h"
#include "gpio.h"
#include "usart.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_CH_CENTER 1000   // SBUS中位值
#define SBUS_CH_RANGE  1600    // SBUS有效范围（减去死区）
#define DEADZONE       150      // 摇杆死区
uint16_t failsafe_status;     /*0=正常   1=信号丢失*/
uint16_t SBUS_footer;     /* 帧尾 0x00，留作后期校验 */
uint8_t buf[25];           /* DMA 一次性搬进来的 25 B 原始 SBUS 帧 */
uint16_t CH[18];          /* 解析后的 16 路模拟通道 + 2 路数字通道 */
uint8_t L;
uint8_t R;
/* 以下 4 个变量用来「逐字节找帧头」 */
extern uint8_t pre_catch_buf[1];      /* 每次只收 1 B，用来找 0x0F */
int pre_init_counter = 0;
bool start_counting = false;
bool catch_signal = false;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    /* 1. 如果还没找到帧头，就逐字节判断 -------------------- */
    if (pre_catch_buf[0] == 0x0F)   /* SBUS 帧头固定 0x0F */
        start_counting = true;       /* 找到帧头，开始计数 */

    if (pre_init_counter < 24 && start_counting == true)   /* 先凑够 24 B，让 DMA 指针刚好落在下一帧的帧头位置 */
    {
        pre_init_counter++;
        return;
    }
    /* 2. 一旦凑够 24 B，就切换成「整帧 DMA」模式 ------------ */
    else if (catch_signal == false)
    {
        catch_signal = true;        /* 标记正式进入解析阶段 */
        HAL_UART_AbortReceive(&huart1);       /* 停掉单字节模式 */
        HAL_UART_Receive_DMA(&huart1, buf, 25);    /* 循环收整帧 */
        return;                          /* 第一次不解析数据 */
    }
    /* 3. 正常解析 25 B SBUS 帧 ------------------------------ */
    if (buf[0] == 0x0F)           /* 帧头不对，直接丢弃 */
    {
        /* 3.1 16 路 11-bit 通道位段拼接
		     *  字节:  1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22
		     *  通道: |CH1        |CH2        |CH3        |CH4        |CH5        |CH6 ...
		     *  说明：按 SBUS 协议，低字节在前，且跨字节不对齐，需要多次移位拼接。
		     * -------------------------------------------------------*/

        CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF;
        CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF;
        CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF;
        CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF;
        CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF;
        CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF;
        CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF;
        CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF;
        CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF;
        CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF;
        CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF;
        CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF;
        CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF;
        CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF;
        CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF;
        CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF;
        /* 3.2 2 路数字通道（第 23 字节的 bit0/bit1） */

        if (buf[23] & (1 << 0))
        {
            CH[16] = 1;
        }
        else
        {
            CH[16] = 0;
        }

        if (buf[23] & (1 << 1))
        {
            CH[17] = 1;
        }
        else
        {
            CH[17] = 0;
        }
        /* 3.3 Failsafe 标志位（第 23 字节的 bit2/bit3） */


        failsafe_status = SBUS_SIGNAL_OK;  /* 先假设正常 */
        if (buf[23] & (1 << 2))
        {
            failsafe_status = SBUS_SIGNAL_LOST;
        }

        if (buf[23] & (1 << 3))
        {
            failsafe_status = SBUS_SIGNAL_FAILSAFE;
        }
        /* 3.4 保存帧尾，留作后期扩展校验 */

        SBUS_footer = buf[24];
        /* 期望 0x00 */

        /* 解析完成，主循环里可以直接读 CH[0..17] 和 failsafe_status */

    }
    else
    {
        
        start_counting = false;
        catch_signal = false;
        pre_init_counter = 0;
        HAL_UART_AbortReceive(&huart1);
        HAL_UART_Receive_DMA(&huart1, pre_catch_buf, 1);
        return;              /* 等待下一次 DMA 完成中断 */
    }
}
void SBUS_UpdateChassis(Chassis_Motion_t* chassis_motion)
{
    // 1. 读取原始值并转换为有符号浮点
    float ch0_raw = (float)CH[0] - SBUS_CH_CENTER;
    float ch1_raw = (float)CH[1] - SBUS_CH_CENTER;
    float ch3_raw = (float)CH[3] - SBUS_CH_CENTER;

    // 2. 死区处理
    if (fabsf(ch0_raw) < DEADZONE) ch0_raw = 0.0f;
    if (fabsf(ch1_raw) < DEADZONE) ch1_raw = 0.0f;
    if (fabsf(ch3_raw) < DEADZONE) ch3_raw = 0.0f;

// 3. 归一化到[-1000, 1000]范围（与ChassisSpeed_t结构匹配）
    chassis_motion->vx = -(ch1_raw / SBUS_CH_RANGE) * 12.0f;  // 前后速度
    chassis_motion->vy = -(ch0_raw / SBUS_CH_RANGE) * 12.0f;  // 左右速度
//    chassis->rotate = -(ch3_raw / SBUS_CH_RANGE) * 3000.0f;
    
   // 4. 限制最大值

    if (chassis_motion->vx > 6.0f) chassis_motion->vx = 6.0f;
    if (chassis_motion->vx < -6.0f) chassis_motion->vx = -6.0f;
    if (chassis_motion->vy > 6.0f) chassis_motion->vy = 6.0f;
    if (chassis_motion->vy < -6.0f) chassis_motion->vy = -6.0f;
		if (chassis_motion->vy > 0.0f) {L=0;R=1;};
    if (chassis_motion->vy < 0.0f) {L=1;R=0;};
//		if (chassis->rotate > 3000.0f) chassis->rotate = 3000.0f;    
//    if (chassis->rotate < -3000.0f) chassis->rotate = -3000.0f;  
}
