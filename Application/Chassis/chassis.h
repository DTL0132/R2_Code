#ifndef CHASSIS_H
#define CHASSIS_H

#include "dvc_motor.h"
#include "drv_can.h"
#include "sbus.h"
#define WHEEL_RADIUS     0.15f
#define MOTOR_GEAR_RATIO 19.0f
#define WHEEL_BASE       0.52f				
#define TRACK_WIDTH      0.48f      
#define PI (3.14159265358979323846f)
#define L_PLUS_W 1.0f
/* 摩擦前馈系数 (粘性摩擦 + 库仑摩擦) */
#define FRICTION_FF_V 60.0f   /* 粘性摩擦系数, 与目标速度成正比 */
#define FRICTION_FF_C 100.0f   /* 库仑摩擦系数, 补偿静摩擦力 */

/* 3508 电机实例（定义在 chassis.c） */
extern struct Class_Motor_C620 motor_3508_1;
extern struct Class_Motor_C620 motor_3508_2;
extern struct Class_Motor_C620 motor_3508_3;
extern struct Class_Motor_C620 motor_3508_4;
/* 夹爪 3508（CAN ID5，反馈 0x205，控制帧 0x1FF） */
extern struct Class_Motor_C620 motor_gripper;

/* 初始化 4 路 3508 电机：速度环 PID + OMEGA 模式 + 目标速度归零 */
void Chassis_3508_Omega_Init(void);

/* 初始化 4 路 3508 电机：速度环 + 角度环 PID + ANGLE 模式 + 目标角度归零 */
void Chassis_3508_Angle_Init(void);

/* CAN 接收回调：根据 CAN ID 将数据分发到对应的 3508 电机 */
void Chassis_3508_CAN_RxCallback(Struct_CAN_Rx_Buffer *Rx_Buffer);

/* 设置 4 路 3508 电机的目标角速度 (rad/s) */
void Chassis_3508_Set_Target_Omegas(float w1, float w2, float w3, float w4);
void Chassis_3508_Set_Target_Angles(float a1, float a2, float a3, float a4);
/* 运行 4 路 3508 电机的 PID 计算并输出到 CAN 发送缓冲区 */
void Chassis_3508_PID_Run(void);

/* 夹爪：初始化 / CAN 接收 / 周期控制 */
void Gripper_3508_Init(void);
void Gripper_CAN_RxCallback(Struct_CAN_Rx_Buffer *Rx_Buffer);
void Gripper_Task(void);

/* 获取 motor_3508_1 的调试数据，存入 buf[0..3]：
 *   buf[0] = Target_Omega
 *   buf[1] = Rx_Omega
 *   buf[2] = Now_Torque
 *   buf[3] = Out
 */
void Chassis_3508_Get_VOFA_Data(float *buf);


void Mecanum_Inverse_Kinematics(Chassis_Motion_t *chassis);
void Chassis_Param_Init(Chassis_Motion_t *chassis);
void up_task(void);

/* ==================== 航向保持 (IMU 辅助) ==================== */

/* 航向保持默认参数 */
#define HEADING_KP_DEFAULT       3.0f   /* 角速率阻尼 P 增益 */
#define HEADING_KI_DEFAULT       0.5f   /* 累积漂移修正 I 增益 */
#define HEADING_MAX_CORRECTION   2.0f   /* 最大修正 wz (rad/s) */
#define HEADING_DEADZONE         0.02f  /* 死区 (rad/s), 约 1°/s, 忽略微小抖动 */

/**
 * @brief 航向保持控制器
 *
 * 前进/后退时用陀螺仪抑制左右偏移:
 *   P 项阻尼瞬时旋转, I 项修正累积航向漂移
 */
typedef struct
{
    float heading_error;   /* 累积航向误差 = ∫gyro_z*dt (应为 0) */
    float kp;              /* 速率阻尼 P 增益 */
    float ki;              /* 漂移修正 I 增益 */
    float max_correction;  /* 最大修正 wz */
    float correction;      /* 当前修正输出 (rad/s) */
} ChassisHeading_t;

void ChassisHeading_Init(ChassisHeading_t *h);
void ChassisHeading_Reset(ChassisHeading_t *h);  /* 清零累积误差 (阶段切换时调用) */
float ChassisHeading_Update(ChassisHeading_t *h, float gyro_z, float dt);

#endif
