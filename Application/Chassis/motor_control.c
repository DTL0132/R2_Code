#include "motor_control.h"
#include "mecanum.h"
#include "CAN_receive.h"
#include "main.h"
#include "PID.h"

// 目标速度（从遥控器设置）
//static float target_vx = 0, target_vy = 0, target_omega = 0;
//pid_type_def motor_speed_pid[4] ={0};

//#define M3505_MOTOR_SPEED_PID_KP 3.08f
//#define M3505_MOTOR_SPEED_PID_KI 0.265f
//#define M3505_MOTOR_SPEED_PID_KD 0.111f
//fp32 speed_pid[3] = { M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD };

//#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
//#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//// 如果想先不用全向轮运动学解析，就先调试下面的代码
//#define M3508_SPEED_SET_1    100
//#define M3508_SPEED_SET_2    100
//#define M3508_SPEED_SET_3    100
//#define M3508_SPEED_SET_4    100
//fp32 motor_speed_set[4] = { M3508_SPEED_SET_1,M3508_SPEED_SET_2, M3508_SPEED_SET_3 ,M3508_SPEED_SET_4 };


/**
 * 设置机器人目标速度
 * vx: 前进为正 (m/s)
 * vy: 左移为正 (m/s)
 * omega: 逆时针为正 rad/s
 */
//void set_robot_velocity(float vx, float vy, float omega)
//{
//    target_vx = vx;
//    target_vy = vy;
//    target_omega = omega;
//}


//void motor_init(void)
//{
//	for ( int m = 0; m < 4; m++ )
//	{
//	  PID_init( &motor_speed_pid[m], PID_POSITION, speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT );
//	}
//}

//// 该函数是接入遥控之前的
//void mecanum_control_task(void)
//{
//	// 先进行PID初始化
//	
//    // 1. 读取当前电机转速
//    float current_rpm[4];
//    for (int i = 0; i < 4; i++) 
//	{
//		// 使用get_chassis_motor_measure_point函数
//		const motor_measure_t *motor_ptr = get_chassis_motor_measure_point(i);
//		if (motor_ptr != NULL) {
//			current_rpm[i] = (float)motor_ptr->speed_rpm;
//		} else {
//			current_rpm[i] = 0.0f;  // 安全处理
//		}
//	}
//    // 2. 运动学逆解：计算机器人速度对应的四个轮子目标转速
//		float target_rpm[4];
//    mecanum_inverse_kinematics(target_vx, target_vy, target_omega, target_rpm);
//    
//	
//    // 3. PID计算控制量
//    int16_t motor_cmd[4];
//    for (int i = 0; i < 4; i++) {
//        float pid_out = PID_calc( &motor_speed_pid[i], current_rpm[i], target_rpm[i]);
//        motor_cmd[i] = (int16_t)pid_out;
//    }
//    
//    // 4. 发送CAN指令
//    CAN_cmd_chassis(motor_cmd[0], motor_cmd[1], motor_cmd[2], motor_cmd[3]);
//}
