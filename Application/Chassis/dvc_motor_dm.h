/**
 * @file dvc_motor_dm.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 达妙电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DVC_MOTOR_DM_H
#define DVC_MOTOR_DM_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"
#include "drv_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机状态
 *
 */
typedef enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
} Enum_Motor_DM_Status;

/**
 * @brief 达妙电机的ID枚举类型, 一拖四模式用
 *
 */
typedef enum Enum_Motor_DM_Motor_ID_1_To_4
{
    Motor_DM_ID_0x301 = 1,
    Motor_DM_ID_0x302,
    Motor_DM_ID_0x303,
    Motor_DM_ID_0x304,
    Motor_DM_ID_0x305,
    Motor_DM_ID_0x306,
    Motor_DM_ID_0x307,
    Motor_DM_ID_0x308,
} Enum_Motor_DM_Motor_ID_1_To_4;

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
typedef enum Enum_Motor_DM_Control_Status_Normal
{
    Motor_DM_Control_Status_DISABLE = 0x0,
    Motor_DM_Control_Status_ENABLE,
    Motor_DM_Control_Status_OVERVOLTAGE = 0x8,
    Motor_DM_Control_Status_UNDERVOLTAGE,
    Motor_DM_Control_Status_OVERCURRENT,
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,
    Motor_DM_Control_Status_LOSE_CONNECTION,
    Motor_DM_Control_Status_MOS_OVERLOAD,
} Enum_Motor_DM_Control_Status_Normal;

/**
 * @brief 达妙电机控制方式
 *
 */
typedef enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
    Motor_DM_Control_Method_1_TO_4_CURRENT,
    Motor_DM_Control_Method_1_TO_4_OMEGA,
    Motor_DM_Control_Method_1_TO_4_ANGLE,
} Enum_Motor_DM_Control_Method;

/**
 * @brief 达妙电机传统模式源数据
 *
 */
struct Struct_Motor_DM_CAN_Rx_Data_Normal
{
    uint8_t CAN_ID : 4;
    uint8_t Control_Status_Enum : 4;
    uint16_t Angle_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机一拖四模式源数据
 *
 */
struct Struct_Motor_DM_CAN_Rx_Data_1_To_4
{
    uint16_t Encoder_Reverse;
    // 角速度100倍
    int16_t Omega_Reverse;
    // 电流值, mA
    int16_t Current_Reverse;
    uint8_t Rotor_Temperature;
    uint8_t MOS_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT
{
    uint16_t Control_Angle_Reverse;
    uint8_t Control_Omega_11_4;
    uint8_t Control_Omega_3_0_K_P_11_8;
    uint8_t K_P_7_0;
    uint8_t K_D_11_4;
    uint8_t K_D_3_0_Control_Torque_11_8;
    uint8_t Control_Torque_7_0;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega
{
    float Control_Angle;
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega
{
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, EMIT控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT
{
    float Control_Angle;
    // 限定速度用, rad/s的100倍
    uint16_t Control_Omega;
    // 限定电流用, 电流最大值的10000倍
    uint16_t Control_Current;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 传统模式有效
 *
 */
struct Struct_Motor_DM_Rx_Data_Normal
{
    enum Enum_Motor_DM_Control_Status_Normal Control_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief 达妙电机经过处理的数据, 一拖四模式有效
 *
 */
struct Struct_Motor_DM_Rx_Data_1_To_4
{
    float Now_Angle;
    float Now_Omega;
    float Now_Current;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief Reusable, 达妙电机, 传统模式
 * 没有零点, 可在上位机调零点
 * 初始化的角度, 角速度, 扭矩, 电流等参数是J4310电机默认值
 *
 */
struct Class_Motor_DM_Normal
{
    // 初始化相关变量

    // 绑定的CAN
    struct Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t CAN_Rx_ID;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t CAN_Tx_ID;
    // 最大位置, 与上位机控制幅值PMAX保持一致
    float Angle_Max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float Omega_Max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float Torque_Max;
    // 最大电流, 与上位机串口中上电打印电流保持一致
    float Current_Max;

    // 常量

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag;

    // 发送缓冲区
    uint8_t Tx_Data[8];

    // 读变量

    // 电机状态
    enum Enum_Motor_DM_Status Motor_DM_Status;
    // 电机对外接口信息
    struct Struct_Motor_DM_Rx_Data_Normal Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    enum Enum_Motor_DM_Control_Method Motor_DM_Control_Method;

    // 角度, rad, 目标角度
    float Control_Angle;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float Control_Omega;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float Control_Torque;
    // 电流, A, EMIT模式是限幅, 其余模式无效
    float Control_Current;
    // K_P, 0~500, MIT模式有效
    float K_P;
    // K_D, 0~5, MIT模式有效
    float K_D;
};

/**
 * @brief Reusable, 达妙电机, 一拖四模式
 * 没有零点, 可在上位机调零点
 *
 */
struct Class_Motor_DM_1_To_4
{
    // PID角度环控制
    struct Class_PID PID_Angle;
    // PID角速度环控制
    struct Class_PID PID_Omega;

    // 初始化相关变量

    // 绑定的CAN
    struct Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 达妙系列0x301~0x308
    enum Enum_Motor_DM_Motor_ID_1_To_4 CAN_Rx_ID;
    // 编码器偏移
    int32_t Encoder_Offset;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 最大电流
    float Current_Max;

    // 常量

    // 一圈编码器刻度
    uint16_t Encoder_Num_Per_Round;

    // 电流到输出的转化系数
    float Current_To_Out;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag;
    // 输出量
    float Out;

    // 读变量

    // 电机状态
    enum Enum_Motor_DM_Status Motor_DM_Status;
    // 电机对外接口信息
    struct Struct_Motor_DM_Rx_Data_1_To_4 Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    enum Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
    // 目标的角度
    float Target_Angle;
    // 目标的速度, rad/s
    float Target_Omega;
    // 目标的电流, A
    float Target_Current;
    // 前馈的速度, rad/s
    float Feedforward_Omega;
    // 前馈的电流, A
    float Feedforward_Current;
};

typedef struct Class_Motor_DM_Normal Class_Motor_DM_Normal;
typedef struct Class_Motor_DM_1_To_4 Class_Motor_DM_1_To_4;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

// Class_Motor_DM_Normal
void Class_Motor_DM_Normal_Init(struct Class_Motor_DM_Normal *this_motor,
    CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
    enum Enum_Motor_DM_Control_Method __Motor_DM_Control_Method,
    float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Current_Max);

void Class_Motor_DM_Normal_CAN_RxCpltCallback(struct Class_Motor_DM_Normal *this_motor, uint8_t *Rx_Data);

void Class_Motor_DM_Normal_CAN_Send_Clear_Error(struct Class_Motor_DM_Normal *this_motor);

void Class_Motor_DM_Normal_CAN_Send_Enter(struct Class_Motor_DM_Normal *this_motor);

void Class_Motor_DM_Normal_CAN_Send_Exit(struct Class_Motor_DM_Normal *this_motor);

void Class_Motor_DM_Normal_CAN_Send_Save_Zero(struct Class_Motor_DM_Normal *this_motor);

void Class_Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(struct Class_Motor_DM_Normal *this_motor);

void Class_Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(struct Class_Motor_DM_Normal *this_motor);

// Class_Motor_DM_1_To_4
void Class_Motor_DM_1_To_4_Init(struct Class_Motor_DM_1_To_4 *this_motor,
    CAN_HandleTypeDef *hcan, enum Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID,
    enum Enum_Motor_DM_Control_Method __Motor_DM_Control_Method,
    int32_t __Encoder_Offset, float __Current_Max);

void Class_Motor_DM_1_To_4_CAN_RxCpltCallback(struct Class_Motor_DM_1_To_4 *this_motor, uint8_t *Rx_Data);

void Class_Motor_DM_1_To_4_TIM_100ms_Alive_PeriodElapsedCallback(struct Class_Motor_DM_1_To_4 *this_motor);

void Class_Motor_DM_1_To_4_TIM_1ms_Calculate_PeriodElapsedCallback(struct Class_Motor_DM_1_To_4 *this_motor);

// --- Inline accessor functions for Class_Motor_DM_Normal ---

/**
 * @brief 获取角度最大值
 *
 * @return float 角度最大值
 */
static inline float Class_Motor_DM_Normal_Get_Angle_Max(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Angle_Max);
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */
static inline float Class_Motor_DM_Normal_Get_Omega_Max(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Omega_Max);
}

/**
 * @brief 获取扭矩最大值
 *
 * @return float 扭矩最大值
 */
static inline float Class_Motor_DM_Normal_Get_Torque_Max(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Torque_Max);
}

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
static inline float Class_Motor_DM_Normal_Get_Current_Max(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Current_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return enum Enum_Motor_DM_Status 电机状态
 */
static inline enum Enum_Motor_DM_Status Class_Motor_DM_Normal_Get_Status(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Motor_DM_Status);
}

/**
 * @brief 获取电机控制状态
 *
 * @return enum Enum_Motor_DM_Control_Status_Normal 电机控制状态
 */
static inline enum Enum_Motor_DM_Control_Status_Normal Class_Motor_DM_Normal_Get_Control_Status(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Rx_Data.Control_Status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
static inline float Class_Motor_DM_Normal_Get_Now_Angle(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
static inline float Class_Motor_DM_Normal_Get_Now_Omega(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
static inline float Class_Motor_DM_Normal_Get_Now_Torque(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
static inline float Class_Motor_DM_Normal_Get_Now_MOS_Temperature(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Rx_Data.Now_MOS_Temperature);
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
static inline float Class_Motor_DM_Normal_Get_Now_Rotor_Temperature(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Rx_Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return enum Enum_Motor_DM_Control_Method 电机控制方式
 */
static inline enum Enum_Motor_DM_Control_Method Class_Motor_DM_Normal_Get_Control_Method(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Motor_DM_Control_Method);
}

/**
 * @brief 获取角度, rad, 目标角度
 *
 * @return float 角度, rad, 目标角度
 */
static inline float Class_Motor_DM_Normal_Get_Control_Angle(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Control_Angle);
}

/**
 * @brief 获取角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @return float 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
static inline float Class_Motor_DM_Normal_Get_Control_Omega(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Control_Omega);
}

/**
 * @brief 获取扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @return float 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
static inline float Class_Motor_DM_Normal_Get_Control_Torque(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Control_Torque);
}

/**
 * @brief 获取电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @return float 电流, A, EMIT模式是限幅, 其余模式无效
 */
static inline float Class_Motor_DM_Normal_Get_Control_Current(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->Control_Current);
}

/**
 * @brief 获取K_P, 0~500, MIT模式有效
 *
 * @return float K_P, 0~500, MIT模式有效
 */
static inline float Class_Motor_DM_Normal_Get_K_P(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->K_P);
}

/**
 * @brief 获取K_D, 0~5, MIT模式有效
 *
 * @return float K_D, 0~5, MIT模式有效
 */
static inline float Class_Motor_DM_Normal_Get_K_D(struct Class_Motor_DM_Normal *this_motor)
{
    return (this_motor->K_D);
}

/**
 * @brief 设定角度, rad, 目标角度
 *
 * @param __Control_Angle 角度, rad, 目标角度
 */
static inline void Class_Motor_DM_Normal_Set_Control_Angle(struct Class_Motor_DM_Normal *this_motor, float __Control_Angle)
{
    this_motor->Control_Angle = __Control_Angle;
}

/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @param __Control_Omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
static inline void Class_Motor_DM_Normal_Set_Control_Omega(struct Class_Motor_DM_Normal *this_motor, float __Control_Omega)
{
    this_motor->Control_Omega = __Control_Omega;
}

/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @param __Control_Torque 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
static inline void Class_Motor_DM_Normal_Set_Control_Torque(struct Class_Motor_DM_Normal *this_motor, float __Control_Torque)
{
    this_motor->Control_Torque = __Control_Torque;
}

/**
 * @brief 设定电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @param __Control_Current 电流, A, EMIT模式是限幅, 其余模式无效
 */
static inline void Class_Motor_DM_Normal_Set_Control_Current(struct Class_Motor_DM_Normal *this_motor, float __Control_Current)
{
    this_motor->Control_Current = __Control_Current;
}

/**
 * @brief 设定K_P, 0~500, MIT模式有效
 *
 * @param __K_P K_P, 0~500, MIT模式有效
 */
static inline void Class_Motor_DM_Normal_Set_K_P(struct Class_Motor_DM_Normal *this_motor, float __K_P)
{
    this_motor->K_P = __K_P;
}

/**
 * @brief 设定K_D, 0~5, MIT模式有效
 *
 * @param __K_D K_D, 0~5, MIT模式有效
 */
static inline void Class_Motor_DM_Normal_Set_K_D(struct Class_Motor_DM_Normal *this_motor, float __K_D)
{
    this_motor->K_D = __K_D;
}

// --- Inline accessor functions for Class_Motor_DM_1_To_4 ---

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
static inline float Class_Motor_DM_1_To_4_Get_Current_Max(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Current_Max);
}

/**
 * @brief 获取理论最大输出电流
 *
 * @return float 理论最大输出电流
 */
static inline float Class_Motor_DM_1_To_4_Get_Theoretical_Output_Current_Max(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Theoretical_Output_Current_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return enum Enum_Motor_DM_Status 电机状态
 */
static inline enum Enum_Motor_DM_Status Class_Motor_DM_1_To_4_Get_Status(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Motor_DM_Status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
static inline float Class_Motor_DM_1_To_4_Get_Now_Angle(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
static inline float Class_Motor_DM_1_To_4_Get_Now_Omega(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前电流
 *
 * @return float 当前电流
 */
static inline float Class_Motor_DM_1_To_4_Get_Now_Current(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Rx_Data.Now_Current);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
static inline float Class_Motor_DM_1_To_4_Get_Now_MOS_Temperature(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Rx_Data.Now_MOS_Temperature);
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
static inline float Class_Motor_DM_1_To_4_Get_Now_Rotor_Temperature(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Rx_Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return enum Enum_Motor_DM_Control_Method 电机控制方式
 */
static inline enum Enum_Motor_DM_Control_Method Class_Motor_DM_1_To_4_Get_Control_Method(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Motor_DM_Control_Method);
}

/**
 * @brief 获取目标的角度
 *
 * @return float 目标的角度
 */
static inline float Class_Motor_DM_1_To_4_Get_Target_Angle(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
static inline float Class_Motor_DM_1_To_4_Get_Target_Omega(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Target_Omega);
}

/**
 * @brief 获取目标的电流, A
 *
 * @return float 目标的电流, A
 */
static inline float Class_Motor_DM_1_To_4_Get_Target_Current(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Target_Current);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
static inline float Class_Motor_DM_1_To_4_Get_Feedforward_Omega(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Feedforward_Omega);
}

/**
 * @brief 获取前馈的电流, A
 *
 * @return float 前馈的电流, A
 */
static inline float Class_Motor_DM_1_To_4_Get_Feedforward_Current(struct Class_Motor_DM_1_To_4 *this_motor)
{
    return (this_motor->Feedforward_Current);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DM_Motor_Control_Method 电机控制方式
 */
static inline void Class_Motor_DM_1_To_4_Set_Control_Method(struct Class_Motor_DM_1_To_4 *this_motor, enum Enum_Motor_DM_Control_Method __DM_Motor_Control_Method)
{
    this_motor->Motor_DM_Control_Method = __DM_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度
 *
 * @param __Target_Angle 目标的角度
 */
static inline void Class_Motor_DM_1_To_4_Set_Target_Angle(struct Class_Motor_DM_1_To_4 *this_motor, float __Target_Angle)
{
    this_motor->Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
static inline void Class_Motor_DM_1_To_4_Set_Target_Omega(struct Class_Motor_DM_1_To_4 *this_motor, float __Target_Omega)
{
    this_motor->Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的电流, A
 *
 * @param __Target_Current 目标的电流, A
 */
static inline void Class_Motor_DM_1_To_4_Set_Target_Current(struct Class_Motor_DM_1_To_4 *this_motor, float __Target_Current)
{
    this_motor->Target_Current = __Target_Current;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
static inline void Class_Motor_DM_1_To_4_Set_Feedforward_Omega(struct Class_Motor_DM_1_To_4 *this_motor, float __Feedforward_Omega)
{
    this_motor->Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的电流, A
 *
 * @param __Feedforward_Current 前馈的电流, A
 */
static inline void Class_Motor_DM_1_To_4_Set_Feedforward_Current(struct Class_Motor_DM_1_To_4 *this_motor, float __Feedforward_Current)
{
    this_motor->Feedforward_Current = __Feedforward_Current;
}

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
