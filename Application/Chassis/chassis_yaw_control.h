/**
 * @file chassis_yaw_control.h
 * @brief 底盘 Yaw 角闭环旋转控制器 (基于 IMU 陀螺仪反馈)
 * @version 1.0
 * @date 2026-06-06
 *
 * 使用 P 控制器将底盘精确旋转到目标 yaw 角。
 * 配合 MPU6500 陀螺仪 z 轴角速度积分实现闭环。
 */

#ifndef CHASSIS_YAW_CONTROL_H
#define CHASSIS_YAW_CONTROL_H

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include "imu_mpu6500.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* 默认 P 增益: 每 rad 误差输出多少 rad/s 的 wz */
#define YAW_KP_DEFAULT         12.0f

/* 默认 I 增益: 消除稳态静差, 小角度也能转到位 */
#define YAW_KI_DEFAULT         0.5f

/* I 项积分限幅 (rad), 防止积分饱和 */
#define YAW_I_MAX              1.0f

/* 默认最大旋转角速度 (rad/s), 防止飞车 */
#define YAW_MAX_WZ_DEFAULT     4.0f

/* 默认到位阈值 (rad), ~1.15°, 小角度也能精确触发 */
#define YAW_THRESHOLD_DEFAULT  0.02f

/* 刹车参数 */
#define YAW_BRAKE_SPEED        0.8f    /* 刹车反向速度 (rad/s) */
#define YAW_BRAKE_TICKS        20      /* 刹车时长 (×10ms = 200ms) */
#define YAW_ROTATE_SPEED       2.0f    /* 旋转速度 (rad/s), P 控制器限幅内的兜底值 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Yaw 控制器状态
 */
typedef enum
{
    YAW_IDLE = 0,       /* 未启动 */
    YAW_ROTATING,       /* 正在旋转 */
    YAW_DONE            /* 旋转完成 */
} YawControl_State_t;

/**
 * @brief Yaw 闭环旋转控制器
 */
typedef struct
{
    YawControl_State_t  state;
    float               target_yaw;
    float               kp;
    float               ki;
    float               i_max;
    float               error_i;       /* 误差积分累积 */
    float               max_wz;
    float               threshold;
    float               current_wz;
} YawController_t;

/**
 * @brief 底盘旋转完整状态机
 *
 * IDLE → ROTATING (P控制) → BRAKING (反向刹车) → DONE (清零锁定)
 */
typedef enum
{
    ROTATE_IDLE = 0,
    ROTATE_RUNNING,
    ROTATE_BRAKING,
    ROTATE_DONE
} ChassisRotate_State_t;

typedef struct
{
    ChassisRotate_State_t state;
    YawController_t       ctrl;
    float                 yaw;
    float                 brake_dir;    /* 刹车方向: +1=CCW, -1=CW */
    uint32_t              brake_tick;
    bool                  imu_ready;
} ChassisRotate_t;

/* Exported function declarations --------------------------------------------*/

void YawController_Init(YawController_t *ctrl);
void YawController_StartRotation(YawController_t *ctrl, float current_yaw, float angle_delta);
bool YawController_Update(YawController_t *ctrl, float current_yaw);
float YawController_GetWz(const YawController_t *ctrl);

/**
 * @brief 初始化底盘旋转模块 (绑定 IMU, 校准陀螺仪)
 * @param rt   旋转句柄
 * @param imu  IMU 句柄 (需已 MPU6500_Init)
 */
void ChassisRotate_Init(ChassisRotate_t *rt, MPU6500_t *imu);

/**
 * @brief 触发旋转指定角度
 *
 * 正角度 = 顺时针, 负角度 = 逆时针 (俯视)
 * 范围: -360° ~ +360°
 *
 * @param rt      旋转句柄
 * @param degrees 旋转角度 (度), +CW, -CCW
 */
void ChassisRotate_ByAngle(ChassisRotate_t *rt, float degrees);

/* 保留兼容旧代码的宏 */
#define ChassisRotate_90CW(rt)   ChassisRotate_ByAngle((rt),  90.0f)
#define ChassisRotate_90CCW(rt)  ChassisRotate_ByAngle((rt), -90.0f)

/**
 * @brief 每周期调用一次, 执行旋转状态机
 *
 * 包含: 陀螺仪读取 → yaw 积分 → P 控制 → 刹车 → 清零锁定
 * 调用者需在外层执行 Chassis_3508_PID_Run() + TIM_CAN_PeriodElapsedCallback()
 *
 * @param rt   旋转句柄
 * @param imu  IMU 句柄
 * @return 当前 wz 指令, 调用者写入电机 (ROTATING/BRAKING 时有效)
 */
float ChassisRotate_Update(ChassisRotate_t *rt, MPU6500_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_YAW_CONTROL_H */
