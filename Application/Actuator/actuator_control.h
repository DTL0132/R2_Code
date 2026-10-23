/**
 * @file actuator_control.h
 * @brief 双DM4310执行器控制模块
 */

#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "dvc_motor_dm.h"
#include "drv_can.h"

/* 4310 电机实例（定义在 actuator_control.c） */
extern Class_Motor_DM_Normal motor_4310_1;
extern Class_Motor_DM_Normal motor_4310_2;

void Actuator_Init(void);
void Actuator_Set_Control(float angle1, float omega1, float angle2, float omega2);
void Actuator_CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *Rx_Buffer);
void Actuator_KeepAlive(void);
void Actuator_Unitree_Init(void);
void Actuator_Unitree_Default_Pos(void);

#endif
