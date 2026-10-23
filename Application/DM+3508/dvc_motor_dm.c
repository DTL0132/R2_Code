/**
 * @file dvc_motor_dm.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief 达妙电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 清除电机错误信息, 传统模式有效
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xfb};
// 使能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xfc};
// 失能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Exit[8] = {0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xfd};
// 保存当前电机位置为零点, 传统模式有效
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xfe};

/* Private function declarations ---------------------------------------------*/

static void Class_Motor_DM_Normal_Data_Process(struct Class_Motor_DM_Normal *this_motor);
static void Class_Motor_DM_Normal_Output(struct Class_Motor_DM_Normal *this_motor);

static void Class_Motor_DM_1_To_4_Data_Process(struct Class_Motor_DM_1_To_4 *this_motor);
static void Class_Motor_DM_1_To_4_PID_Calculate(struct Class_Motor_DM_1_To_4 *this_motor);
static void Class_Motor_DM_1_To_4_Output(struct Class_Motor_DM_1_To_4 *this_motor);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区, 一拖四模式有效
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
static uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, enum Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID_1_To_4)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x301):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x302):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x303):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x304):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[6]);

            break;
        }
        case (Motor_DM_ID_0x305):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x306):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x307):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x308):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[6]);

            break;
        }
        }
    }
    // else if (hcan == &hcan2)
    // {
    //     switch (__CAN_Rx_ID_1_To_4)
    //     {
    //     case (Motor_DM_ID_0x301):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[0]);

    //         break;
    //     }
    //     case (Motor_DM_ID_0x302):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[2]);

    //         break;
    //     }
    //     case (Motor_DM_ID_0x303):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[4]);

    //         break;
    //     }
    //     case (Motor_DM_ID_0x304):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[6]);

    //         break;
    //     }
    //     case (Motor_DM_ID_0x305):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[0]);

    //         break;
    //     }
    //     case (Motor_DM_ID_0x306):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[2]);

    //         break;
    //     }
    //     case (Motor_DM_ID_0x307):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[4]);

    //         break;
    //     }
    //     case (Motor_DM_ID_0x308):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[6]);

    //         break;
    //     }
    //     }
    // }
    return (tmp_tx_data_ptr);
}

// ==================== Class_Motor_DM_Normal ====================

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_DM_Control_Method 电机控制方式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Class_Motor_DM_Normal_Init(struct Class_Motor_DM_Normal *this_motor, CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, enum Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Current_Max)
{
    if (hcan->Instance == CAN1)
    {
        this_motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        this_motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }

    this_motor->CAN_Rx_ID = __CAN_Rx_ID;
    switch (__Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
    {
        this_motor->CAN_Tx_ID = __CAN_Tx_ID;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
    {
        this_motor->CAN_Tx_ID = __CAN_Tx_ID + 0x100;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_OMEGA):
    {
        this_motor->CAN_Tx_ID = __CAN_Tx_ID + 0x200;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_EMIT):
    {
        this_motor->CAN_Tx_ID = __CAN_Tx_ID + 0x300;
        break;
    }
    }
    this_motor->Motor_DM_Control_Method = __Motor_DM_Control_Method;
    this_motor->Angle_Max = __Angle_Max;
    this_motor->Omega_Max = __Omega_Max;
    this_motor->Torque_Max = __Torque_Max;
    this_motor->Current_Max = __Current_Max;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_DM_Normal_CAN_RxCpltCallback(struct Class_Motor_DM_Normal *this_motor, uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    this_motor->Flag += 1;

    Class_Motor_DM_Normal_Data_Process(this_motor);
}

/**
 * @brief 发送清除错误信息
 *
 */
void Class_Motor_DM_Normal_CAN_Send_Clear_Error(struct Class_Motor_DM_Normal *this_motor)
{
    CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, DM_Motor_CAN_Message_Clear_Error, 8);
}

/**
 * @brief 发送使能电机
 *
 */
void Class_Motor_DM_Normal_CAN_Send_Enter(struct Class_Motor_DM_Normal *this_motor)
{
    CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, DM_Motor_CAN_Message_Enter, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void Class_Motor_DM_Normal_CAN_Send_Exit(struct Class_Motor_DM_Normal *this_motor)
{
    CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, DM_Motor_CAN_Message_Exit, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void Class_Motor_DM_Normal_CAN_Send_Save_Zero(struct Class_Motor_DM_Normal *this_motor)
{
    CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Class_Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(struct Class_Motor_DM_Normal *this_motor)
{
    // 判断该时间段内是否接收过电机数据
    if (this_motor->Flag == this_motor->Pre_Flag)
    {
        // 电机断开连接
        this_motor->Motor_DM_Status = Motor_DM_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        this_motor->Motor_DM_Status = Motor_DM_Status_ENABLE;
    }

    this_motor->Pre_Flag = this_motor->Flag;

    if(this_motor->Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        Class_Motor_DM_Normal_CAN_Send_Enter(this_motor);
    }
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void Class_Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(struct Class_Motor_DM_Normal *this_motor)
{
    if (this_motor->Rx_Data.Control_Status == Motor_DM_Status_ENABLE)
    {
        // 电机在线, 正常控制
        Math_Constrain(&this_motor->Control_Angle, -this_motor->Angle_Max, this_motor->Angle_Max);
        Math_Constrain(&this_motor->Control_Omega, -this_motor->Omega_Max, this_motor->Omega_Max);
        Math_Constrain(&this_motor->Control_Torque, -this_motor->Torque_Max, this_motor->Torque_Max);
        Math_Constrain(&this_motor->Control_Current, -this_motor->Current_Max, this_motor->Current_Max);
        Math_Constrain(&this_motor->K_P, 0.0f, 500.0f);
        Math_Constrain(&this_motor->K_D, 0.0f, 5.0f);

        Class_Motor_DM_Normal_Output(this_motor);
    }
    else if (this_motor->Rx_Data.Control_Status == Motor_DM_Status_DISABLE)
    {
        // 电机可能掉线, 使能电机
        Class_Motor_DM_Normal_CAN_Send_Enter(this_motor);
    }
    else
    {
        // 电机错误, 发送清除错误帧
        Class_Motor_DM_Normal_CAN_Send_Clear_Error(this_motor);
    }
}

/**
 * @brief 数据处理过程
 *
 */
static void Class_Motor_DM_Normal_Data_Process(struct Class_Motor_DM_Normal *this_motor)
{
    // 数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    struct Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer = (struct Struct_Motor_DM_CAN_Rx_Data_Normal *)this_motor->CAN_Manage_Object->Rx_Buffer.Data;

    // 电机ID不匹配, 则不进行处理
    if(tmp_buffer->CAN_ID != (this_motor->CAN_Tx_ID & 0x0f))
    {
        return;
    }

    // 处理大小端
    Math_Endian_Reverse_16_SrcDst((void *)&tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

    this_motor->Rx_Data.Control_Status = (enum Enum_Motor_DM_Control_Status_Normal)(tmp_buffer->Control_Status_Enum);

    // 计算圈数与总角度值
    delta_encoder = tmp_encoder - this_motor->Rx_Data.Pre_Encoder;
    if (delta_encoder < -(1 << 15))
    {
        // 正方向转过了一圈
        this_motor->Rx_Data.Total_Round++;
    }
    else if (delta_encoder > (1 << 15))
    {
        // 反方向转过了一圈
        this_motor->Rx_Data.Total_Round--;
    }
    this_motor->Rx_Data.Total_Encoder = this_motor->Rx_Data.Total_Round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);

    // 计算电机本身信息
    this_motor->Rx_Data.Now_Angle = (float)(this_motor->Rx_Data.Total_Encoder) / (float)((1 << 16) - 1) * this_motor->Angle_Max * 2.0f;
    this_motor->Rx_Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, this_motor->Omega_Max);
    this_motor->Rx_Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, this_motor->Torque_Max);
    this_motor->Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    this_motor->Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    this_motor->Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
static void Class_Motor_DM_Normal_Output(struct Class_Motor_DM_Normal *this_motor)
{
    // 电机控制
    switch (this_motor->Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
    {
        struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer = (struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *)this_motor->Tx_Data;

        uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

        tmp_angle = Math_Float_To_Int(this_motor->Control_Angle, 0, this_motor->Angle_Max, 0x7fff, (1 << 16) - 1);
        tmp_omega = Math_Float_To_Int(this_motor->Control_Omega, 0, this_motor->Omega_Max, 0x7ff, (1 << 12) - 1);
        tmp_torque = Math_Float_To_Int(this_motor->Control_Torque, 0, this_motor->Torque_Max, 0x7ff, (1 << 12) - 1);
        tmp_k_p = Math_Float_To_Int(this_motor->K_P, 0, 500.0f, 0, (1 << 12) - 1);
        tmp_k_d = Math_Float_To_Int(this_motor->K_D, 0, 5.0f, 0, (1 << 12) - 1);

        tmp_buffer->Control_Angle_Reverse = Math_Endian_Reverse_16_SrcDst(&tmp_angle, NULL);
        tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
        tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
        tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
        tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

        CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, this_motor->Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
    {
        struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer = (struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *)this_motor->Tx_Data;

        tmp_buffer->Control_Angle = this_motor->Control_Angle;
        tmp_buffer->Control_Omega = this_motor->Control_Omega;

        CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, this_motor->Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_OMEGA):
    {
        struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *tmp_buffer = (struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *)this_motor->Tx_Data;

        tmp_buffer->Control_Omega = this_motor->Control_Omega;

        CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, this_motor->Tx_Data, 4);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_EMIT):
    {
        struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *tmp_buffer = (struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *)this_motor->Tx_Data;

        tmp_buffer->Control_Angle = this_motor->Control_Angle;
        tmp_buffer->Control_Omega = (uint16_t)(this_motor->Control_Omega * 100.0f);
        tmp_buffer->Control_Current = (uint16_t)(this_motor->Control_Current / this_motor->Current_Max * 10000.0f);

        CAN_Send_Data(this_motor->CAN_Manage_Object->CAN_Handler, this_motor->CAN_Tx_ID, this_motor->Tx_Data, 8);

        break;
    }
    }
}

// ==================== Class_Motor_DM_1_To_4 ====================

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DM_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Current_Max 最大电流
 */
void Class_Motor_DM_1_To_4_Init(struct Class_Motor_DM_1_To_4 *this_motor, CAN_HandleTypeDef *hcan, enum Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID, enum Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, int32_t __Encoder_Offset, float __Current_Max)
{
    this_motor->Encoder_Num_Per_Round = 8192;
    this_motor->Current_To_Out = 16384.0f / 10.261194f;
    this_motor->Theoretical_Output_Current_Max = 10.261194f;

    if (hcan->Instance == CAN1)
    {
        this_motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        this_motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    this_motor->CAN_Rx_ID = __CAN_Rx_ID;
    this_motor->Motor_DM_Control_Method = __Motor_DM_Control_Method;
    this_motor->Encoder_Offset = __Encoder_Offset;
    this_motor->Current_Max = __Current_Max;
    this_motor->Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID);
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_DM_1_To_4_CAN_RxCpltCallback(struct Class_Motor_DM_1_To_4 *this_motor, uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    this_motor->Flag += 1;

    Class_Motor_DM_1_To_4_Data_Process(this_motor);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DM_1_To_4_TIM_100ms_Alive_PeriodElapsedCallback(struct Class_Motor_DM_1_To_4 *this_motor)
{
    // 判断该时间段内是否接收过电机数据
    if (this_motor->Flag == this_motor->Pre_Flag)
    {
        // 电机断开连接
        this_motor->Motor_DM_Status = Motor_DM_Status_DISABLE;
        Class_PID_Set_Integral_Error(&this_motor->PID_Angle, 0.0f);
        Class_PID_Set_Integral_Error(&this_motor->PID_Omega, 0.0f);
    }
    else
    {
        // 电机保持连接
        this_motor->Motor_DM_Status = Motor_DM_Status_ENABLE;
    }
    this_motor->Pre_Flag = this_motor->Flag;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Motor_DM_1_To_4_TIM_1ms_Calculate_PeriodElapsedCallback(struct Class_Motor_DM_1_To_4 *this_motor)
{
    float tmp_value;

    Class_Motor_DM_1_To_4_PID_Calculate(this_motor);

    tmp_value = this_motor->Target_Current + this_motor->Feedforward_Current;
    Math_Constrain(&tmp_value, -this_motor->Current_Max, this_motor->Current_Max);
    this_motor->Out = tmp_value * this_motor->Current_To_Out;

    Class_Motor_DM_1_To_4_Output(this_motor);

    this_motor->Feedforward_Current = 0.0f;
    this_motor->Feedforward_Omega = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 */
static void Class_Motor_DM_1_To_4_Data_Process(struct Class_Motor_DM_1_To_4 *this_motor)
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    struct Struct_Motor_DM_CAN_Rx_Data_1_To_4 *tmp_buffer = (struct Struct_Motor_DM_CAN_Rx_Data_1_To_4 *) this_motor->CAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端
    Math_Endian_Reverse_16_SrcDst((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16_SrcDst((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16_SrcDst((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - this_motor->Rx_Data.Pre_Encoder;
    if (delta_encoder < -this_motor->Encoder_Num_Per_Round / 2)
    {
        // 正方向转过了一圈
        this_motor->Rx_Data.Total_Round++;
    }
    else if (delta_encoder > this_motor->Encoder_Num_Per_Round / 2)
    {
        // 反方向转过了一圈
        this_motor->Rx_Data.Total_Round--;
    }
    this_motor->Rx_Data.Total_Encoder = this_motor->Rx_Data.Total_Round * this_motor->Encoder_Num_Per_Round + tmp_encoder + this_motor->Encoder_Offset;

    // 计算电机本身信息
    this_motor->Rx_Data.Now_Angle = (float) this_motor->Rx_Data.Total_Encoder / (float) this_motor->Encoder_Num_Per_Round * 2.0f * PI;
    this_motor->Rx_Data.Now_Omega = tmp_omega / 100.0f * RPM_TO_RADPS;
    this_motor->Rx_Data.Now_Current = tmp_current / 1000.0f;
    this_motor->Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    this_motor->Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    this_motor->Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
static void Class_Motor_DM_1_To_4_PID_Calculate(struct Class_Motor_DM_1_To_4 *this_motor)
{
    switch (this_motor->Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_1_TO_4_CURRENT):
    {
        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_OMEGA):
    {
        Class_PID_Set_Target(&this_motor->PID_Omega, this_motor->Target_Omega + this_motor->Feedforward_Omega);
        Class_PID_Set_Now(&this_motor->PID_Omega, this_motor->Rx_Data.Now_Omega);
        Class_PID_TIM_Adjust_PeriodElapsedCallback(&this_motor->PID_Omega);

        this_motor->Target_Current = Class_PID_Get_Out(&this_motor->PID_Omega);

        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_ANGLE):
    {
        Class_PID_Set_Target(&this_motor->PID_Angle, this_motor->Target_Angle);
        Class_PID_Set_Now(&this_motor->PID_Angle, this_motor->Rx_Data.Now_Angle);
        Class_PID_TIM_Adjust_PeriodElapsedCallback(&this_motor->PID_Angle);

        this_motor->Target_Omega = Class_PID_Get_Out(&this_motor->PID_Angle);

        Class_PID_Set_Target(&this_motor->PID_Omega, this_motor->Target_Omega + this_motor->Feedforward_Omega);
        Class_PID_Set_Now(&this_motor->PID_Omega, this_motor->Rx_Data.Now_Omega);
        Class_PID_TIM_Adjust_PeriodElapsedCallback(&this_motor->PID_Omega);

        this_motor->Target_Current = Class_PID_Get_Out(&this_motor->PID_Omega);

        break;
    }
    default:
    {
        this_motor->Target_Current = 0.0f;

        break;
    }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
static void Class_Motor_DM_1_To_4_Output(struct Class_Motor_DM_1_To_4 *this_motor)
{
    *(int16_t *) this_motor->Tx_Data = (int16_t)(this_motor->Out);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
