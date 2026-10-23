//
// Created by user on 2023/4/24.
//

#ifndef _DM_H_
#define _DM_H_

#include "main.h"

#define DM_CAN &hcan2
//MIT模式参数初始化
#define MIT_KP 100//48.2
#define MIP_KD 1.04
#define MIT_INIT_KP 5
#define MIT_INIT_KD 1


typedef struct {
    int ID; //电机id
    int ERR; //故障类型
    float POS; //位置信息
    float VEL; //速度信息
    float T; //扭矩信息
    float T_MOS; //MOS的平均温度
    float T_Rotor; //电机内部线圈的平均温度
}DM_Info;

extern DM_Info dm1;
extern DM_Info Send_DM;
void DM_MOTOR_CONTROL(void);

void DM_MOTOR_START(CAN_HandleTypeDef *hcan, uint16_t id);

void DM_PID_DRIVE(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel);

void DM_MIT_DRIVE(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);

void SAVE_ZERO_POSITION(CAN_HandleTypeDef *hcan, uint16_t id);

void DM_MIT_SPEMODE(float VEL,float TQ);

void DM_MIT_POSMODE(float POS,float KP,float TQ);

void DM_MI_TQMODE(float TQ);

void DM_MIT_POSInit(float POS);

void DM_PID_POSInit(void);

void DM_PID_POSMODE(void);

void DM(float angle,float vel);  //DM赋值

extern int float_to_uint(float x, float x_min, float x_max, int bits);


#endif //CTEST_DM_H
