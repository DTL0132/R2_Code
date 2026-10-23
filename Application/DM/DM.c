//
// Created by user on 2023/4/24.
//
/*******************************************************************************
 *
 * TITR-ROBOCON-2023 驱动库(HAL)
 * 创建时间 2026/5/9
 * 创建人 CBC
 *
 * ****************************************************************************/

#include "DM.h"
#include "can.h"

#define CAN_ID 0x01
#define PID_CONTROL // 三种控制方式 MIT_CONTROL PID_CONTROL VEL_CONTROL

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

uint8_t DM_MOTOR_INIT = 0; // 0代表电机未初始化 1代表电机初始化成功

DM_Info dm1;
DM_Info Send_DM;

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DM_MOTOR_CONTROL()
{

    // 下面是使能函数
    if (DM_MOTOR_INIT == 0)
    {
#ifdef MIT_CONTROL
        DM_MOTOR_START(DM_CAN, CAN_ID);
#endif

#ifdef PID_CONTROL
        DM_MOTOR_START(DM_CAN, CAN_ID + 0x100);

        // DM_PID_DRIVE(DM_CAN, CAN_ID + 0x100, 0.0f, 2.0f);
#endif

#ifdef VEL_CONTROL
        DM_MOTOR_START(DM_CAN, CAN_ID + 0x200);
#endif
        DM_MOTOR_INIT = 1;
    }

    // 下面是控制函数
#ifdef MIT_CONTROL
    DM_MIT_DRIVE(DM_CAN, CAN_ID, 0.0F, 2.0f, 0.0, 0.0, 0.0);
#endif

#ifdef PID_CONTROL
    DM_PID_DRIVE(DM_CAN, CAN_ID + 0x100, Send_DM.POS, Send_DM.VEL); // 弧度制 代表输出转轴的弧度    -3.14   2
#endif

#ifdef VEL_CONTROL
    void DM_VEL_DRIVE(DM_CAN, CAN_ID + 0x200, 0.0f);
#endif
}

// 下面定义了DM电机使能函数

void DM_MOTOR_START(CAN_HandleTypeDef *hcan, uint16_t id)
{
    uint32_t send_mail_box;     // 发送邮箱
    uint8_t can_data_buffer[8]; // can发送数据缓存
    CAN_TxHeaderTypeDef CanTxMsg;

    CanTxMsg.StdId = id;
    CanTxMsg.IDE = CAN_ID_STD;
    CanTxMsg.RTR = CAN_RTR_DATA;
    CanTxMsg.DLC = 0x08;
    can_data_buffer[0] = 0xFF;
    can_data_buffer[1] = 0xFF;
    can_data_buffer[2] = 0xFF;
    can_data_buffer[3] = 0xFF;
    can_data_buffer[4] = 0xFF;
    can_data_buffer[5] = 0xFF;
    can_data_buffer[6] = 0xFF;
    can_data_buffer[7] = 0xFC;

    HAL_CAN_AddTxMessage(hcan, &CanTxMsg, can_data_buffer, &send_mail_box);
}

// 传统PID模式驱动
void DM_PID_DRIVE(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel)
{
    uint32_t send_mail_box;     // 发送邮箱
    uint8_t can_data_buffer[8]; // can发送数据缓存
    CAN_TxHeaderTypeDef CanTxMsg;

    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;

    CanTxMsg.StdId = id;
    CanTxMsg.IDE = CAN_ID_STD;
    CanTxMsg.RTR = CAN_RTR_DATA;
    CanTxMsg.DLC = 0x08;
    can_data_buffer[1] = *(pbuf + 1);
    can_data_buffer[2] = *(pbuf + 2);
    can_data_buffer[3] = *(pbuf + 3);
    can_data_buffer[4] = *vbuf;
    can_data_buffer[5] = *(vbuf + 1);
    can_data_buffer[6] = *(vbuf + 2);
    can_data_buffer[7] = *(vbuf + 3);

    HAL_CAN_AddTxMessage(hcan, &CanTxMsg, can_data_buffer, &send_mail_box);
}

// MIT模式驱动
void DM_MIT_DRIVE(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    uint32_t send_mail_box;     // 发送邮箱
    uint8_t can_data_buffer[8]; // can发送数据缓存
    CAN_TxHeaderTypeDef CanTxMsg;

    CanTxMsg.StdId = id;
    CanTxMsg.IDE = CAN_ID_STD;
    CanTxMsg.RTR = CAN_RTR_DATA;
    CanTxMsg.DLC = 0x08;
    can_data_buffer[0] = (pos_tmp >> 8);
    can_data_buffer[1] = pos_tmp;
    can_data_buffer[2] = (vel_tmp >> 4);
    can_data_buffer[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    can_data_buffer[4] = kp_tmp;
    can_data_buffer[5] = (kd_tmp >> 4);
    can_data_buffer[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    can_data_buffer[7] = tor_tmp;

    HAL_CAN_AddTxMessage(hcan, &CanTxMsg, can_data_buffer, &send_mail_box);
}

// 速度模式
void DM_VEL_DRIVE(CAN_HandleTypeDef *hcan, uint16_t id, float _vel)
{
    uint8_t *vbuf;
    vbuf = (uint8_t *)&_vel;
    uint32_t send_mail_box;     // 发送邮箱
    uint8_t can_data_buffer[8]; // can发送数据缓存
    CAN_TxHeaderTypeDef CanTxMsg;

    CanTxMsg.StdId = id;
    CanTxMsg.IDE = CAN_ID_STD;
    CanTxMsg.RTR = CAN_RTR_DATA;
    CanTxMsg.DLC = 0x04;
    can_data_buffer[0] = *vbuf;
    can_data_buffer[1] = *(vbuf + 1);
    can_data_buffer[2] = *(vbuf + 2);
    can_data_buffer[3] = *(vbuf + 3);

    HAL_CAN_AddTxMessage(hcan, &CanTxMsg, can_data_buffer, &send_mail_box);
}
