/**
 * @file chassis_yaw_control.c
 * @brief 底盘 Yaw 角闭环旋转控制器实现
 * @version 1.0
 * @date 2026-06-06
 */

/* Includes ------------------------------------------------------------------*/

#include "chassis_yaw_control.h"
#include "chassis.h"   /* PI 宏定义 */

/* Private defines -----------------------------------------------------------*/

#define TWO_PI 6.28318530717958647692f

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 角度归一化到 [-PI, PI]
 * @param angle  任意角度 (rad)
 * @return 归一化后的角度
 */
static float Yaw_Normalize(float angle)
{
    while (angle > PI)
    {
        angle -= TWO_PI;
    }
    while (angle < -PI)
    {
        angle += TWO_PI;
    }
    return angle;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 Yaw 控制器
 */
void YawController_Init(YawController_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->state      = YAW_IDLE;
    ctrl->target_yaw = 0.0f;
    ctrl->kp         = YAW_KP_DEFAULT;
    ctrl->max_wz     = YAW_MAX_WZ_DEFAULT;
    ctrl->threshold  = YAW_THRESHOLD_DEFAULT;
    ctrl->current_wz = 0.0f;
}

/**
 * @brief 启动相对角度旋转
 */
void YawController_StartRotation(YawController_t *ctrl, float current_yaw, float angle_delta)
{
    if (ctrl == NULL)
    {
        return;
    }

    /* 计算目标 yaw = 当前值 + 偏移量, 归一化防止边界溢出 */
    ctrl->target_yaw = Yaw_Normalize(current_yaw + angle_delta);
    ctrl->state      = YAW_ROTATING;
    ctrl->current_wz = 0.0f;
}

/**
 * @brief 执行一次控制器迭代
 *
 * @return true  旋转完成
 * @return false 仍在旋转或空闲
 */
bool YawController_Update(YawController_t *ctrl, float current_yaw)
{
    float error;

    if (ctrl == NULL || ctrl->state != YAW_ROTATING)
    {
        return (ctrl != NULL && ctrl->state == YAW_DONE);
    }

    /* 最短路径角误差 */
    error = ctrl->target_yaw - current_yaw;
    error = Yaw_Normalize(error);

    /* 死区判断: 误差小于阈值则停止 */
    if (error < ctrl->threshold && error > -ctrl->threshold)
    {
        ctrl->current_wz = 0.0f;
        ctrl->state = YAW_DONE;
        return true;
    }

    /* P 控制器: wz = Kp * error */
    ctrl->current_wz = ctrl->kp * error;

    /* 限幅 */
    if (ctrl->current_wz > ctrl->max_wz)
    {
        ctrl->current_wz = ctrl->max_wz;
    }
    else if (ctrl->current_wz < -ctrl->max_wz)
    {
        ctrl->current_wz = -ctrl->max_wz;
    }

    return false;
}

/**
 * @brief 获取当前 wz 指令
 */
float YawController_GetWz(const YawController_t *ctrl)
{
    if (ctrl == NULL)
    {
        return 0.0f;
    }
    return ctrl->current_wz;
}
