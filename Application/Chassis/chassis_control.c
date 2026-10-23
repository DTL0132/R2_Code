//#include "chassis_control.h"
//#include "sbus.h"
//void Mecanum_Inverse_Kinematics(Chassis_Motion_t *chassis)
//{
//    float wheel_line_speed[4] = {0};
//    wheel_line_speed[0] = -chassis->vx -chassis->vy + L_PLUS_W /2* chassis->wz;
//    wheel_line_speed[1] = -chassis->vx +chassis->vy + L_PLUS_W /2* chassis->wz;
//    wheel_line_speed[2] = chassis->vx  +chassis->vy + L_PLUS_W /2* chassis->wz;
//    wheel_line_speed[3] = chassis->vx -chassis->vy  + L_PLUS_W /2* chassis->wz;
//    for(int i=0; i<4; i++)
//    { 
//        chassis->motor_target_speed[i] = (wheel_line_speed[i] * 10.0f * MOTOR_GEAR_RATIO) / (2 * PI * WHEEL_RADIUS);
//    }
//}
//void Chassis_Param_Init(Chassis_Motion_t *chassis)
//{  
//    chassis->vx = 0.0f;
//    chassis->vy = 0.0f;
//    chassis->wz = 0.0f;
//    for(int i=0; i<4; i++) chassis->motor_target_speed[i] = 0.0f;
//}
