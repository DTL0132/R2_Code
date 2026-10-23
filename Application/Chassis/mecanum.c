#include "mecanum.h"
#include "math.h"

// 机器人参数（根据实际测量修改！）
//#define WHEEL_RADIUS      0.076f    // 轮子半径(m)
//#define ROBOT_RADIUS      0.13433f      // 轮子到中心距离(m)
//#define GEAR_RATIO        19.0f     // 3508减速比
//#define PI 				  3.14159f  // 定义PI
//float angle = (float)(PI / 4.0f);
///**
// * 全向轮运动学逆解（X型麦轮布置）
// * 轮子顺序：1前右, 2前左, 3后左, 4后右
// */
//void mecanum_inverse_kinematics(float vx, float vy, float omega, float wheel_rpm[4]) // omega是角速度，控制转速
//{
//    // 将m/s转换为RPM的系数（转/分钟）
//    float rpm_factor = 60.0f * GEAR_RATIO / (2.0f * PI * WHEEL_RADIUS);
//    
//    // 旋转分量
//    float rot_factor = omega * ROBOT_RADIUS/* * (sqrt(2.0f) / 2)*/;
//	
//    // 四个轮子的线速度(m/s)
//    wheel_rpm[0] = ( vx * cos(angle) - vy * cos(angle) + rot_factor) * rpm_factor;  // 前右
//    wheel_rpm[1] = ( vx * cos(angle) + vy * cos(angle) + rot_factor) * rpm_factor;  // 前左
//    wheel_rpm[2] = (-vx * cos(angle) + vy * cos(angle) + rot_factor) * rpm_factor;  // 后左
//    wheel_rpm[3] = (-vx * cos(angle) - vy * cos(angle) + rot_factor) * rpm_factor;  // 后右
//}
//void SBUS_UpdateChassis(ChassisSpeed_t* chassis)
//{
//		chassis->x =  3000.0f;  // 前后速度
//    chassis->y =  3000.0f;  // 左右速度
//    chassis->rotate = 3000.0f;  // 旋转速度
//    if (chassis->x > 3000.0f) chassis->x = 3000.0f;
//    if (chassis->x < -3000.0f) chassis->x = -3000.0f;
//    if (chassis->y > 3000.0f) chassis->y = 3000.0f;
//    if (chassis->y < -3000.0f) chassis->y = -3000.0f;
//		if (chassis->rotate > 3000.0f) chassis->rotate = 3000.0f;  
//    if (chassis->rotate < -3000.0f) chassis->rotate = -3000.0f;  
//}
