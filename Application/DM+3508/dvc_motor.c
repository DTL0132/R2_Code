/**
 * @file dvc_motor.c
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief CAN电机配置与操作
 * @version 0.1
 * @date 2022-08-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor.h"
#include "chassis.h"
#include "pid.h"
#include "struct_typedef.h"
/* Private macros -----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
static uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, enum Enum_CAN_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (CAN_Motor_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
        }
        break;
        case (CAN_Motor_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
        }
        break;
        case (CAN_Motor_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
        }
        break;
        case (CAN_Motor_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
        }
        break;
        case (CAN_Motor_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
        }
        break;
        case (CAN_Motor_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
        }
        break;
        case (CAN_Motor_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
        }
        break;
        case (CAN_Motor_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
        }
        break;
        case (CAN_Motor_ID_0x209):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
        }
        break;
        case (CAN_Motor_ID_0x20A):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
        }
        break;
        case (CAN_Motor_ID_0x20B):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
        }
        break;
        }
    }
    // else if (hcan == &hcan2)
    // {
    //     switch (__CAN_ID)
    //     {
    //     case (CAN_Motor_ID_0x201):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x202):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x203):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x204):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x205):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x206):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x207):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x208):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x209):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[0]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x20A):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[2]);
    //     }
    //     break;
    //     case (CAN_Motor_ID_0x20B):
    //     {
    //         tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[4]);
    //     }
    //     break;
    //     }
    // }
    return (tmp_tx_data_ptr);
}


// ==================== C620 ====================

/**
 * @brief 输出到CAN总线
 *
 */
static void C620_Output(struct Class_Motor_C620 *this_motor)
{
    this_motor->CAN_Tx_Data[0] = (int16_t)this_motor->Out >> 8;
    this_motor->CAN_Tx_Data[1] = (int16_t)this_motor->Out;
}

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __Control_Method 电机控制方式, 默认速度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max 最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 */
void Class_Motor_C620_Init(struct Class_Motor_C620 *this_motor, CAN_HandleTypeDef *hcan, enum Enum_CAN_Motor_ID __CAN_ID, enum Enum_Control_Method __Control_Method, float __Gearbox_Rate, float __Torque_Max)
{
    this_motor->Encoder_Num_Per_Round = 8192;
    this_motor->Output_Max = 16384;

    if (hcan->Instance == CAN1)
    {
        this_motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        this_motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    this_motor->CAN_ID = __CAN_ID;
    this_motor->Control_Method = __Control_Method;
    this_motor->Gearbox_Rate = __Gearbox_Rate;
    this_motor->Torque_Max = __Torque_Max;
    this_motor->CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_Motor_C620_Output(struct Class_Motor_C620 *this_motor)
{
    C620_Output(this_motor);
}

/**
 * @brief 获取最大输出电流
 *
 * @return uint16_t 最大输出电流
 */
uint16_t Class_Motor_C620_Get_Output_Max(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return enum Enum_CAN_Motor_Status 电机状态
 */
enum Enum_CAN_Motor_Status Class_Motor_C620_Get_CAN_Motor_Status(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->CAN_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_Motor_C620_Get_Now_Angle(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_Motor_C620_Get_Now_Omega(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_Motor_C620_Get_Now_Torque(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_Motor_C620_Get_Now_Temperature(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return enum Enum_Control_Method 电机控制方式
 */
enum Enum_Control_Method Class_Motor_C620_Get_Control_Method(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_Motor_C620_Get_Target_Angle(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_Motor_C620_Get_Target_Omega(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_Motor_C620_Get_Target_Torque(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_Motor_C620_Get_Out(struct Class_Motor_C620 *this_motor)
{
    return (this_motor->Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
void Class_Motor_C620_Set_Control_Method(struct Class_Motor_C620 *this_motor, enum Enum_Control_Method __Control_Method)
{
    this_motor->Control_Method = __Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_Motor_C620_Set_Target_Angle(struct Class_Motor_C620 *this_motor, float __Target_Angle)
{
    this_motor->Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_Motor_C620_Set_Target_Omega(struct Class_Motor_C620 *this_motor, float __Target_Omega)
{
    this_motor->Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_Motor_C620_Set_Target_Torque(struct Class_Motor_C620 *this_motor, float __Target_Torque)
{
    this_motor->Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Out 输出量
 */
void Class_Motor_C620_Set_Out(struct Class_Motor_C620 *this_motor, float __Out)
{
    this_motor->Out = __Out;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_C620_CAN_RxCpltCallback(struct Class_Motor_C620 *this_motor, uint8_t *Rx_Data)
{
    int16_t delta_encoder;

    this_motor->Flag += 1;

    this_motor->Pre_Encoder = this_motor->Rx_Encoder;

    this_motor->Rx_Encoder = (Rx_Data[0] << 8) | Rx_Data[1];
    this_motor->Rx_Omega = (Rx_Data[2] << 8) | Rx_Data[3];
//    this_motor->Rx_Omega = 1.0f;
    this_motor->Rx_Torque = (Rx_Data[4] << 8) | Rx_Data[5];
    this_motor->Rx_Temperature = Rx_Data[6];
    // 跳过首次接收时的溢出判断, 防止 Pre_Encoder 初始值为 0 导致误判
    if (this_motor->Flag > 1)
    {
        delta_encoder = this_motor->Rx_Encoder - this_motor->Pre_Encoder;
        if (delta_encoder < -4096)
        {
            //正方向转过了一圈
            this_motor->Total_Round++;
        }
        else if (delta_encoder > 4096)
        {
            //反方向转过了一圈
            this_motor->Total_Round--;
        }
    }
    this_motor->Total_Encoder = this_motor->Total_Round * this_motor->Encoder_Num_Per_Round + this_motor->Rx_Encoder;

    this_motor->Now_Angle = (float)this_motor->Total_Encoder / (float)this_motor->Encoder_Num_Per_Round * 2.0f * PI / this_motor->Gearbox_Rate;
    this_motor->Now_Omega = (float)this_motor->Rx_Omega * RPM_TO_RADPS / this_motor->Gearbox_Rate;
    this_motor->Now_Torque = this_motor->Rx_Torque;
    this_motor->Now_Temperature = this_motor->Rx_Temperature;
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_C620_TIM_Alive_PeriodElapsedCallback(struct Class_Motor_C620 *this_motor)
{
    //判断该时间段内是否接收过电机数据
    if (this_motor->Flag == this_motor->Pre_Flag)
    {
        //电机断开连接
        this_motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        PID_clear(&this_motor->PID_Angle);
        PID_clear(&this_motor->PID_Omega);
    }
    else
    {
        //电机保持连接
        this_motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    this_motor->Pre_Flag = this_motor->Flag;
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Motor_C620_TIM_PID_PeriodElapsedCallback(struct Class_Motor_C620 *this_motor)
{
    switch (this_motor->Control_Method)
    {
    case (Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Class_Motor_C620_Set_Out(this_motor, this_motor->Target_Torque / this_motor->Torque_Max * this_motor->Output_Max);
    }
    break;
    case (Control_Method_TORQUE):
    {
        //默认闭环扭矩控制
        Class_Motor_C620_Set_Out(this_motor, this_motor->Target_Torque / this_motor->Torque_Max * this_motor->Output_Max);
    }
    break;
    case (Control_Method_OMEGA):
    {
        PID_calc(&this_motor->PID_Omega, this_motor->Now_Omega, this_motor->Target_Omega);

        Class_Motor_C620_Set_Out(this_motor, this_motor->PID_Omega.out);
    }
    break;
    case (Control_Method_ANGLE):
    {
        PID_calc(&this_motor->PID_Angle, this_motor->Now_Angle, this_motor->Target_Angle);

        this_motor->Target_Omega = this_motor->PID_Angle.out;

        PID_calc(&this_motor->PID_Omega, this_motor->Now_Omega, this_motor->Target_Omega);

        Class_Motor_C620_Set_Out(this_motor, this_motor->PID_Omega.out);
    }
    break;
    default:
    {
        Class_Motor_C620_Set_Out(this_motor, 0.0f);
    }
    break;
    }
    Class_Motor_C620_Output(this_motor);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
