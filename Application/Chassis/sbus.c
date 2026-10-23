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
#define FRAME_HEAD  0x0F
uint16_t failsafe_status;     /*0=正常   1=信号丢失*/
uint16_t SBUS_footer;     /* 帧尾 0x00，留作后期校验 */
uint8_t buf[25];           /* DMA 一次性搬进来的 25 B 原始 SBUS 帧 */
uint16_t CH[18];          /* 解析后的 16 路模拟通道 + 2 路数字通道 */
uint8_t chassis_times=0;
uint8_t chassis_flag=0;        //判断底盘运动状态，速度为0时为0
uint8_t chassis_init=0;        //底盘起步为0
uint8_t chassis_times_y=0;
uint8_t chassis_flag_y=0;        //判断底盘运动状态，速度为0时为0
uint8_t chassis_init_y=0;        //底盘起步为0
int8_t sign=0;
float ch0_raw=0.0f;
float ch1_raw=0.0f;
float ch3_raw=0.0f;

/* 以下 4 个变量用来「逐字节找帧头」 */
uint8_t pre_catch_buf[1];      /* 每次只收 1 B，用来找 0x0F */
int pre_init_counter = 0;
bool start_counting = false;
bool catch_signal = false;


uint8_t rx_buf[RX_BUF_SIZE];    // DMA 
uint8_t tx_buf[8];    // DMA 接收缓冲区
uint8_t rx_init=0;
volatile uint8_t rx_flag = 0;   // 一帧数据接收完成标志
volatile uint16_t rx_len = 0;   // 实际接收到的字节数
Chassis_Motion_t chassis_motion;
extern volatile uint8_t gripper_cmd; /* 由 freertos.c 中的任务处理 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    /*
     * 夹爪串口指令（UART7，115200 8N1）：
     *   发送 ASCII '1' -> 向内夹取
     *   发送 ASCII '0' -> 松开回到全开位置
     * 收到后回发两个相同字节作为确认，然后重新挂 1 字节 DMA。
     * 只在单字节接收阶段处理，避免整帧 DMA 阶段用旧字节误触发。
     */
    if (huart->Instance == UART7 && catch_signal == false)
    {
        uint8_t cur = pre_catch_buf[0];

        if (cur == '0' || cur == '1')
        {
            uint8_t resp[2];
            resp[0] = cur;
            resp[1] = cur;
            HAL_UART_Transmit(&huart7, resp, 2, 50);
            gripper_cmd = (cur == '1') ? 1 : 0;
            HAL_UART_AbortReceive(&huart7);
            HAL_UART_Receive_DMA(&huart7, pre_catch_buf, 1);
            return;
        }

        /* 串口助手常自动带 \r\n：直接丢掉，避免误切到 SBUS 整帧模式 */
        if (cur == '\r' || cur == '\n')
        {
            HAL_UART_AbortReceive(&huart7);
            HAL_UART_Receive_DMA(&huart7, pre_catch_buf, 1);
            return;
        }
    }
    /* 1. 如果还没找到帧头，就逐字节判断 -------------------- */
    if (pre_catch_buf[0] == 0x0F)   /* SBUS 帧头固定 0x0F */
        start_counting = true;       /* 找到帧头，开始计数 */
//    if (pre_init_counter < 1 && start_counting == true)   /* 先凑够 24 B，让 DMA 指针刚好落在下一帧的帧头位置 */
//    {
//        pre_init_counter++;
//        return;
//				
//    }
    /* 2. 一旦凑够 24 B，就切换成「整帧 DMA」模式 ------------ */

			if (catch_signal == false)
    {
				if (rx_init==RX_BUF_SIZE-1){
				rx_init++;
				 HAL_UART_Receive_DMA(&huart7, buf, 25);  
				return;
				}else{
        catch_signal = true;        /* 标记正式进入解析阶段 */
        HAL_UART_AbortReceive(&huart7);       /* 停掉单字节模式 */
        HAL_UART_Receive_DMA(&huart7, rx_buf, RX_BUF_SIZE);    /* 循环收整帧 */
        return;  
				}                        /* 第一次不解析数据 */
				
		}
    /* 3. 正常解析 25 B SBUS 帧 ------------------------------ */
    if (rx_buf[0] == 0x0F)           /* 帧头不对，直接丢弃 */
    {
//        SBUS_footer = rx_buf[3];
					if(!rx_buf[4]){
							sign=1;
					}else{
							sign=-1;
					}
        /* 解析完成，主循环里可以直接读 CH[0..17] 和 failsafe_status */
    }
    else
    {
        
        start_counting = false;
        catch_signal = false;
        pre_init_counter = 0;
        HAL_UART_AbortReceive(&huart7);
        HAL_UART_Receive_DMA(&huart7, pre_catch_buf, 1);
        return;              /* 等待下一次 DMA 完成中断 */
    }
}

//void SBUS_RxCpltHandler(UART_HandleTypeDef* huart)
//{
//    /* Quick check: if a simple gripper command ('1' or '0') is received on UART7,
//       reply with double-byte confirmation ('11' or '00'), set gripper_cmd and rearm DMA. */
//    if (huart->Instance == UART7)
//    {
//        uint8_t cur = pre_catch_buf[0];
//        if (cur == '1' || cur == '0')
//        {
//            uint8_t resp[2];
//            resp[0] = cur;
//            resp[1] = cur; /* send '11' or '00' */
//            HAL_UART_Transmit(&huart7, resp, 2, 50);
//            gripper_cmd = (cur == '1') ? 1 : 0;
//            HAL_UART_AbortReceive(huart);
//            HAL_UART_Receive_DMA(huart, pre_catch_buf, 1);
//            return;
//        }
//    }
//    /* 1. 如果还没找到帧头，就逐字节判断 -------------------- */
//    if (pre_catch_buf[0] == 0x0F)   /* SBUS 帧头固定 0x0F */
//        start_counting = true;       /* 找到帧头，开始计数 */
////    if (pre_init_counter < 1 && start_counting == true)   /* 先凑够 24 B，让 DMA 指针刚好落在下一帧的帧头位置 */
////    {
////        pre_init_counter++;
////        return;
////				
////    }
//    /* 2. 一旦凑够 24 B，就切换成「整帧 DMA」模式 ------------ */

//			if (catch_signal == false)
//    {
//				if (rx_init==RX_BUF_SIZE-1){
//				rx_init++;
//				 HAL_UART_Receive_DMA(&huart7, buf, 25);  
//				return;
//				}else{
//        catch_signal = true;        /* 标记正式进入解析阶段 */
//        HAL_UART_AbortReceive(&huart7);       /* 停掉单字节模式 */
//        HAL_UART_Receive_DMA(&huart7, rx_buf, 4);    /* 循环收整帧 */
//        return;  
//				}                        /* 第一次不解析数据 */
//				
//		}
//    /* 3. 正常解析 25 B SBUS 帧 ------------------------------ */
//    if (rx_buf[0] == 0x0F)           /* 帧头不对，直接丢弃 */
//    {
////        SBUS_footer = rx_buf[3];

//        /* 解析完成，主循环里可以直接读 CH[0..17] 和 failsafe_status */
//    }
//    else
//    {
//        
//        start_counting = false;
//        catch_signal = false;
//        pre_init_counter = 0;
//        HAL_UART_AbortReceive(&huart7);
//        HAL_UART_Receive_DMA(&huart7, pre_catch_buf, 1);
//        return;              /* 等待下一次 DMA 完成中断 */
//    }
//}
//  void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
//{
//    /* 1. 如果还没找到帧头，就逐字节判断 -------------------- */
//    if (pre_catch_buf[0] == 0x0F)   /* SBUS 帧头固定 0x0F */
//        start_counting = true;       /* 找到帧头，开始计数 */

//    if (pre_init_counter < 24 && start_counting == true)   /* 先凑够 24 B，让 DMA 指针刚好落在下一帧的帧头位置 */
//    {
//        pre_init_counter++;
//        return;
//    }
//    /* 2. 一旦凑够 24 B，就切换成「整帧 DMA」模式 ------------ */
//    else if (catch_signal == false)
//    {
//        catch_signal = true;        /* 标记正式进入解析阶段 */
//        HAL_UART_AbortReceive(&huart1);       /* 停掉单字节模式 */
//        HAL_UART_Receive_DMA(&huart1, buf, 25);    /* 循环收整帧 */
//        return;                          /* 第一次不解析数据 */
//    }
//    /* 3. 正常解析 25 B SBUS 帧 ------------------------------ */
//    if (buf[0] == 0x0F)           /* 帧头不对，直接丢弃 */
//    {
//        /* 3.1 16 路 11-bit 通道位段拼接
//		     *  字节:  1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22
//		     *  通道: |CH1        |CH2        |CH3        |CH4        |CH5        |CH6 ...
//		     *  说明：按 SBUS 协议，低字节在前，且跨字节不对齐，需要多次移位拼接。
//		     * -------------------------------------------------------*/

//        CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF;
//        CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF;
//        CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF;
//        CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF;
//        CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF;
//        CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF;
//        CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF;
//        CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF;
//        CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF;
//        CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF;
//        CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF;
//        CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF;
//        CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF;
//        CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF;
//        CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF;
//        CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF;
//        /* 3.2 2 路数字通道（第 23 字节的 bit0/bit1） */

//        if (buf[23] & (1 << 0))
//        {
//            CH[16] = 1;
//        }
//        else
//        {
//            CH[16] = 0;
//        }

//        if (buf[23] & (1 << 1))
//        {
//            CH[17] = 1;
//        }
//        else
//        {
//            CH[17] = 0;
//        }
//        /* 3.3 Failsafe 标志位（第 23 字节的 bit2/bit3） */


//        failsafe_status = SBUS_SIGNAL_OK;  /* 先假设正常 */
//        if (buf[23] & (1 << 2))
//        {
//            failsafe_status = SBUS_SIGNAL_LOST;
//        }

//        if (buf[23] & (1 << 3))
//        {
//            failsafe_status = SBUS_SIGNAL_FAILSAFE;
//        }
//        /* 3.4 保存帧尾，留作后期扩展校验 */

//        SBUS_footer = buf[24];
//        /* 期望 0x00 */

//        /* 解析完成，主循环里可以直接读 CH[0..17] 和 failsafe_status */

//    }
//    else
//    {
//        
//        start_counting = false;
//        catch_signal = false;
//        pre_init_counter = 0;
//        HAL_UART_AbortReceive(&huart1);
//        HAL_UART_Receive_DMA(&huart1, pre_catch_buf, 1);
//        return;              /* 等待下一次 DMA 完成中断 */
//    }
//}

//void Chassis_Communication_Task(Chassis_Motion_t* chassis_motion)
//{
//    //原始数据处理
//    float target_ch0 = (float)CH[0] - SBUS_CH_CENTER;  // 左右
//    float target_ch1 = (float)CH[1] - SBUS_CH_CENTER;  // 前后
//    float target_ch3 = (float)CH[3] - SBUS_CH_CENTER;  // 旋转

//    //死区处理
//    if (fabsf(target_ch0) < DEADZONE) target_ch0 = 0.0f;
//    if (fabsf(target_ch1) < DEADZONE) target_ch1 = 0.0f;
//    if (fabsf(target_ch3) < DEADZONE) target_ch3 = 0.0f;

//    //变化率限制(每周期最多变化 STEP，实现"缓慢起步/刹车")
//    const float STEP = 80.0f;  // 每调用一次最多变化 80，根据调用频率调整
//    float diff0 = target_ch0 - ch0_raw;
//    float diff1 = target_ch1 - ch1_raw;
//    
//    if (fabsf(diff0) > STEP)
//        ch0_raw += (diff0 > 0) ? STEP : -STEP;
//    else
//        ch0_raw = target_ch0;

//    if (fabsf(diff1) > STEP)`
//        ch1_raw += (diff1 > 0) ? STEP : -STEP;
//    else
//        ch1_raw = target_ch1;

//    // ch3 如果需要同样处理，也加上；否则直接赋值
//    ch3_raw = target_ch3;

//    // 4. 归一化输出
//    chassis_motion->vx = -(ch1_raw / SBUS_CH_RANGE) * 12.0f;
//    chassis_motion->vy = -(ch0_raw / SBUS_CH_RANGE) * 12.0f;
//    chassis_motion->wz = -(ch3_raw / SBUS_CH_RANGE) * 3000.0f;

//    // 5. 限幅
//    chassis_motion->vx = fmaxf(-6.0f, fminf(6.0f, chassis_motion->vx));
//    chassis_motion->vy = fmaxf(-6.0f, fminf(6.0f, chassis_motion->vy));
//    chassis_motion->wz = fmaxf(-6.0f, fminf(6.0f, chassis_motion->wz));
//}
void Chassis_Communication_Task(Chassis_Motion_t* chassis_motion)
{
//    chassis_motion->vx = sign*rx_buf[2] /5.0f;
//    chassis_motion->vy = sign*rx_buf[1] /5.0f;
    chassis_motion->wz = sign*rx_buf[3] /5.0f;
    chassis_motion->vx = fmaxf(-6.0f, fminf(6.0f, chassis_motion->vx));
    chassis_motion->vy = fmaxf(-6.0f, fminf(6.0f, chassis_motion->vy));
    chassis_motion->wz = fmaxf(-6.0f, fminf(6.0f, chassis_motion->wz));
}
