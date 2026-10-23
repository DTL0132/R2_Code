/**
 * @file dvc_motor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief CAN电机配置与操作
 * @version 0.1
 * @date 2022-08-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_MOTOR_H
#define DVC_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "pid.h"
#include "drv_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

// RPM换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机状态
 *
 */
enum Enum_CAN_Motor_Status
{
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE,
};

/**
 * @brief CAN电机的ID枚举类型
 *
 */
enum Enum_CAN_Motor_ID
{
    CAN_Motor_ID_UNDEFINED = 0,
    CAN_Motor_ID_0x201,
    CAN_Motor_ID_0x202,
    CAN_Motor_ID_0x203,
    CAN_Motor_ID_0x204,
    CAN_Motor_ID_0x205,
    CAN_Motor_ID_0x206,
    CAN_Motor_ID_0x207,
    CAN_Motor_ID_0x208,
    CAN_Motor_ID_0x209,
    CAN_Motor_ID_0x20A,
    CAN_Motor_ID_0x20B,
};

/**
 * @brief CAN电机的ID分配情况
 *
 */
enum Enum_CAN_Motor_ID_Status
{
    CAN_Motor_ID_Status_FREE = 0,
    CAN_Motor_ID_Status_ALLOCATED,
};

/**
 * @brief 电机控制方式
 *
 */
enum Enum_Control_Method
{
    Control_Method_OPENLOOP = 0,
    Control_Method_TORQUE,
    Control_Method_OMEGA,
    Control_Method_ANGLE,
};


/**
 * @brief C620无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
struct Class_Motor_C620
{
    // PID角度环控制
    pid_type_def PID_Angle;
    // PID角速度环控制
    pid_type_def PID_Omega;

    //初始化相关变量

    //绑定的CAN
    struct Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    enum Enum_CAN_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //减速比, 默认带减速箱
    float Gearbox_Rate;
    //最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
    float Torque_Max;

    //常量

    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round;
    //最大输出扭矩
    uint16_t Output_Max;

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag;

    //接收的编码器位置, 0~8191
    uint16_t Rx_Encoder;
    //接收的速度, rpm
    int16_t Rx_Omega;
    //接收的扭矩, 目标的扭矩, -30000~30000
    int16_t Rx_Torque;
    //接收的温度, 摄氏度
    uint16_t Rx_Temperature;

    //之前的编码器位置
    uint16_t Pre_Encoder;
    //总编码器位置
    int32_t Total_Encoder;
    //总圈数
    int32_t Total_Round;

    //读变量

    //电机状态
    enum Enum_CAN_Motor_Status CAN_Motor_Status;

    //当前的角度, rad
    float Now_Angle;
    //当前的速度, rad/s
    float Now_Omega;
    //当前的扭矩, 直接采用反馈值
    float Now_Torque;
    //当前的温度, 摄氏度
    uint8_t Now_Temperature;

    //写变量

    //读写变量

    //电机控制方式
    enum Enum_Control_Method Control_Method;
    //目标的角度, rad
    float Target_Angle;
    //目标的速度, rad/s
    float Target_Omega;
    //目标的扭矩, 直接采用反馈值
    float Target_Torque;
    //输出量
    float Out;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

// GM6020


void Class_Motor_C620_Init(struct Class_Motor_C620 *this_motor,
    CAN_HandleTypeDef *hcan, enum Enum_CAN_Motor_ID __CAN_ID,
    enum Enum_Control_Method __Control_Method, float __Gearbox_Rate,
    float __Torque_Max);

uint16_t Class_Motor_C620_Get_Output_Max(struct Class_Motor_C620 *this_motor);

enum Enum_CAN_Motor_Status Class_Motor_C620_Get_CAN_Motor_Status(struct Class_Motor_C620 *this_motor);

float Class_Motor_C620_Get_Now_Angle(struct Class_Motor_C620 *this_motor);

float Class_Motor_C620_Get_Now_Omega(struct Class_Motor_C620 *this_motor);

float Class_Motor_C620_Get_Now_Torque(struct Class_Motor_C620 *this_motor);

uint8_t Class_Motor_C620_Get_Now_Temperature(struct Class_Motor_C620 *this_motor);

enum Enum_Control_Method Class_Motor_C620_Get_Control_Method(struct Class_Motor_C620 *this_motor);

float Class_Motor_C620_Get_Target_Angle(struct Class_Motor_C620 *this_motor);

float Class_Motor_C620_Get_Target_Omega(struct Class_Motor_C620 *this_motor);

float Class_Motor_C620_Get_Target_Torque(struct Class_Motor_C620 *this_motor);

float Class_Motor_C620_Get_Out(struct Class_Motor_C620 *this_motor);

void Class_Motor_C620_Set_Control_Method(struct Class_Motor_C620 *this_motor, enum Enum_Control_Method __Control_Method);

void Class_Motor_C620_Set_Target_Angle(struct Class_Motor_C620 *this_motor, float __Target_Angle);

void Class_Motor_C620_Set_Target_Omega(struct Class_Motor_C620 *this_motor, float __Target_Omega);

void Class_Motor_C620_Set_Target_Torque(struct Class_Motor_C620 *this_motor, float __Target_Torque);

void Class_Motor_C620_Set_Out(struct Class_Motor_C620 *this_motor, float __Out);

void Class_Motor_C620_CAN_RxCpltCallback(struct Class_Motor_C620 *this_motor, uint8_t *Rx_Data);

void Class_Motor_C620_TIM_Alive_PeriodElapsedCallback(struct Class_Motor_C620 *this_motor);

void Class_Motor_C620_TIM_PID_PeriodElapsedCallback(struct Class_Motor_C620 *this_motor);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
