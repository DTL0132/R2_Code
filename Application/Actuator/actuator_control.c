/**
 * @file actuator_control.c
 * @brief 双DM4310执行器控制模块
 *
 * 两个DM4310电机: 角度幅值相同, 速度相同, 方向相反
 * 电机1: CAN ID 0x01
 * 电机2: CAN ID 0x02
 */

#include "actuator_control.h"
#include "can.h"
#include "Unitree_control.h"

Class_Motor_DM_Normal motor_4310_1;
Class_Motor_DM_Normal motor_4310_2;

void Actuator_Init(void)
{
    Class_Motor_DM_Normal_Init(&motor_4310_1, &hcan1, 0x01, 0x01,
        Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
        12.5f, 30.0f, 10.0f, 20.0f);
    Class_Motor_DM_Normal_Init(&motor_4310_2, &hcan1, 0x02, 0x02,
        Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
        12.5f, 30.0f, 10.0f, 20.0f);
}

void Actuator_Set_Control(float angle1, float omega1, float angle2, float omega2)
{
    Class_Motor_DM_Normal_Set_Control_Angle(&motor_4310_1, angle1);
    Class_Motor_DM_Normal_Set_Control_Omega(&motor_4310_1, omega1);
    Class_Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&motor_4310_1);

    Class_Motor_DM_Normal_Set_Control_Angle(&motor_4310_2, angle2);
    Class_Motor_DM_Normal_Set_Control_Omega(&motor_4310_2, omega2);
    Class_Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&motor_4310_2);
}

void Actuator_CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        case 0x01:
            Class_Motor_DM_Normal_CAN_RxCpltCallback(&motor_4310_1, Rx_Buffer->Data);
            break;
        case 0x02:
            Class_Motor_DM_Normal_CAN_RxCpltCallback(&motor_4310_2, Rx_Buffer->Data);
            break;
        default:
            break;
    }
}

/**
 * @brief DM 电机存活检测 (每 1ms 调用一次, 内部每 100ms 检测一次)
 */
void Actuator_KeepAlive(void)
{
    static uint32_t counter = 0;

    if (++counter >= 100)
    {
        counter = 0;
        Class_Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&motor_4310_1);
        Class_Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&motor_4310_2);
    }
}

/**
 * @brief  宇树电机初始化 (MIT模式)
 * @param  None
 * @retval None
 */
void Actuator_Unitree_Init(void)
{
    uint8_t ID[4], MODE[4];
    uint8_t z;

    motion_control_init();

    for (z = 0; z < 4; z++)
    {
        ID[z] = z + 1;
        MODE[z] = 1;
    }
    motor_select(ID, MODE);
}

/**
 * @brief  宇树电机默认MIT位置控制 (电机1:上下, 电机2:前后)
 * @param  None
 * @retval None
 */
void Actuator_Unitree_Default_Pos(void)
{
    motor_mit_control(1, 0.10f, 0.03f, 0.10f, -10.0f, 0.5f);
    motor_mit_control(2, 0.10f, 0.03f, 0.10f, -15.0f, 0.5f);
    motion_control_run();
}
