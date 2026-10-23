#include "chassis.h"
#include "chassis_control.h"
#include "sbus.h"
#include "chassis_yaw_control.h"
extern volatile uint8_t gripper_cmd;   /* 夹爪串口指令，定义在 freertos.c */

/* 4 路 3508 电机全局实例 */
struct Class_Motor_C620 motor_3508_1;
struct Class_Motor_C620 motor_3508_2;
struct Class_Motor_C620 motor_3508_3;
struct Class_Motor_C620 motor_3508_4;
struct Class_Motor_C620 motor_gripper;   /* 夹爪 3508，CAN ID5，反馈 0x205 */

/* ==================== 连杆夹爪（3508 + C620，CAN ID5） ==================== */
/*
 * 机械说明：
 *   连杆式夹爪没有固定角度硬限位。上电时先手动掰到最大开度，
 *   代码在第一帧 0x205 反馈到达后，把该位置记录为“全开角”。
 *
 * 控制策略：
 *   1) 向内夹取：OMEGA 速度环低速向内；
 *      当反馈电流超过阈值且转速接近 0 并持续 GRIP_STALL_MS，
 *      判定已经夹住物体，切换到 TORQUE 力矩环输出保持电流。
 *   2) 松爪：ANGLE 角度环回到上电记录的全开角。
 *
 * 单位说明（框架 Class_Motor_C620）：
 *   Now_Angle / Target_Angle  : rad（输出轴，19 减速比已在类内处理）
 *   Now_Omega / Target_Omega  : rad/s（输出轴）
 *   Now_Torque / Target_Torque: 原始反馈电流（±16384 对应 ±20A）
 */
#define GRIP_CLOSE_OMEGA    (+1.0f)    /* 闭合目标转速，正号=向内（按实测方向） */
#define GRIP_STALL_TORQUE    1000.0f   /* 堵转电流阈值，约 1.2A */
#define GRIP_STALL_OMEGA     0.05f     /* 转速低于该值认为停转，rad/s */
#define GRIP_STALL_MS        150U      /* 堵转持续确认时间，ms */
#define GRIP_HOLD_TORQUE    (+1000.0f) /* 夹住后的保持电流，同向内方向 */
#define GRIP_OPEN_TOL        0.02f     /* 回到全开位置的允许误差，rad（约 1.1°） */

typedef enum
{
    GRIP_IDLE = 0,   /* 静止/保持全开 */
    GRIP_CLOSING,    /* 向内夹取中 */
    GRIP_HOLD,       /* 已夹住，力矩保持 */
    GRIP_OPENING     /* 正在回到全开位置 */
} GripperState_t;

static GripperState_t g_grip_state = GRIP_IDLE;
static float    g_grip_open_angle = 0.0f;  /* 上电手动全开位置，rad */
static uint8_t  g_grip_open_valid = 0;     /* 是否已经记录全开角 */
static uint32_t g_grip_stall_tick = 0;     /* 堵转计时起点，0=未计时 */
// uint8_t up_flag=1;
/**
 * @brief 3508 速度环初始化 (OMEGA 模式)
 *
 * 初始化速度 PID + 电机为 OMEGA 控制模式 + 目标速度归零
 */
void Chassis_3508_Omega_Init(void)
{
    /* PID 参数初始化 —— 速度环 */
    /* 内环: 速度→电流, Kp=2500, Ki=0.15, Kd=0.01, max_out=13000 */
    {
        fp32 omega_pid[3] = {2600.0f, 0.15f, 0.01f};
        PID_init(&motor_3508_1.PID_Omega, PID_POSITION, omega_pid, 14000.0f, 8000.0f);
        PID_init(&motor_3508_2.PID_Omega, PID_POSITION, omega_pid, 14000.0f, 8000.0f);
        PID_init(&motor_3508_3.PID_Omega, PID_POSITION, omega_pid, 14000.0f, 8000.0f);
        PID_init(&motor_3508_4.PID_Omega, PID_POSITION, omega_pid, 14000.0f, 8000.0f);
    }

    /* 电机初始化 —— 速度环控制模式 */
    Class_Motor_C620_Init(&motor_3508_1, &hcan1, CAN_Motor_ID_0x201, Control_Method_OMEGA, 19.0f, 16384.0f);
    Class_Motor_C620_Init(&motor_3508_2, &hcan1, CAN_Motor_ID_0x202, Control_Method_OMEGA, 19.0f, 16384.0f);
    Class_Motor_C620_Init(&motor_3508_3, &hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA, 19.0f, 16384.0f);
    Class_Motor_C620_Init(&motor_3508_4, &hcan1, CAN_Motor_ID_0x204, Control_Method_OMEGA, 19.0f, 16384.0f);

    /* 目标速度归零 */
    Class_Motor_C620_Set_Target_Omega(&motor_3508_1, 0.0f);
    Class_Motor_C620_Set_Target_Omega(&motor_3508_2, 0.0f);
    Class_Motor_C620_Set_Target_Omega(&motor_3508_3, 0.0f);
    Class_Motor_C620_Set_Target_Omega(&motor_3508_4, 0.0f);
}

/**
 * @brief 3508 角度环初始化 (ANGLE 模式)
 *
 * 初始化速度 PID + 角度 PID + 电机为 ANGLE 控制模式 + 目标角度归零
 */
void Chassis_3508_Angle_Init(void)
{
    /* PID 参数初始化 —— 速度环 */
    {
        fp32 omega_pid[3] = {2500.0f, 0.15f, 0.01f};
        PID_init(&motor_3508_1.PID_Omega, PID_POSITION, omega_pid, 13000.0f, 8000.0f);
        PID_init(&motor_3508_2.PID_Omega, PID_POSITION, omega_pid, 13000.0f, 8000.0f);
        PID_init(&motor_3508_3.PID_Omega, PID_POSITION, omega_pid, 13000.0f, 8000.0f);
        PID_init(&motor_3508_4.PID_Omega, PID_POSITION, omega_pid, 13000.0f, 8000.0f);
    }

    /* PID 参数初始化 —— 位置环 */
    /* 外环: 角度→目标速度, Ki=0.5*0.01=0.005(匹配10ms控制周期), max_out=30rad/s */
    {
        fp32 angle_pid[3] = {6.0f, 0.01f, 0.0f};
        PID_init(&motor_3508_1.PID_Angle, PID_POSITION, angle_pid, 30.0f, 30.0f);
        PID_init(&motor_3508_2.PID_Angle, PID_POSITION, angle_pid, 30.0f, 30.0f);
        PID_init(&motor_3508_3.PID_Angle, PID_POSITION, angle_pid, 30.0f, 30.0f);
        PID_init(&motor_3508_4.PID_Angle, PID_POSITION, angle_pid, 30.0f, 30.0f);
    }

    /* 电机初始化 —— 位置环控制模式 */
    Class_Motor_C620_Init(&motor_3508_1, &hcan1, CAN_Motor_ID_0x201, Control_Method_ANGLE, 19.0f, 16384.0f);
    Class_Motor_C620_Init(&motor_3508_2, &hcan1, CAN_Motor_ID_0x202, Control_Method_ANGLE, 19.0f, 16384.0f);
    Class_Motor_C620_Init(&motor_3508_3, &hcan1, CAN_Motor_ID_0x203, Control_Method_ANGLE, 19.0f, 16384.0f);
    Class_Motor_C620_Init(&motor_3508_4, &hcan1, CAN_Motor_ID_0x204, Control_Method_ANGLE, 19.0f, 16384.0f);

    /* 目标角度归零 */
    Class_Motor_C620_Set_Target_Angle(&motor_3508_1, 0.0f);
    Class_Motor_C620_Set_Target_Angle(&motor_3508_2, 0.0f);
    Class_Motor_C620_Set_Target_Angle(&motor_3508_3, 0.0f);
    Class_Motor_C620_Set_Target_Angle(&motor_3508_4, 0.0f);
}

void Chassis_3508_CAN_RxCallback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        case (0x201):
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_1, Rx_Buffer->Data);
            break;
        case (0x202):
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_2, Rx_Buffer->Data);
            break;
        case (0x203):
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_3, Rx_Buffer->Data);
            break;
        case (0x204):
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_4, Rx_Buffer->Data);
            break;
    }
}

void Chassis_3508_Set_Target_Omegas(float w1, float w2, float w3, float w4)
{
    Class_Motor_C620_Set_Target_Omega(&motor_3508_1, w1);
    Class_Motor_C620_Set_Target_Omega(&motor_3508_2, w2);
    Class_Motor_C620_Set_Target_Omega(&motor_3508_3, w3);
    Class_Motor_C620_Set_Target_Omega(&motor_3508_4, w4);
  //Class_Motor_C620_Set_Target_Angle(&motor_3508_1, w1);
}
//static void Chassis_Friction_FF(struct Class_Motor_C620 *motor)
//{
//    float ff;
//    float target;
//    target = motor->Target_Omega;
//    ff = FRICTION_FF_V * target;
//    if (target > 0.0f)
//    {
//        ff += FRICTION_FF_C;
//    }
//    else if (target < 0.0f)
//    {
//        ff -= FRICTION_FF_C;
//    }
//    motor->Out += ff;
//}
void Chassis_3508_Set_Target_Angles(float a1, float a2, float a3, float a4)
{
    Class_Motor_C620_Set_Target_Angle(&motor_3508_1, a1);
    Class_Motor_C620_Set_Target_Angle(&motor_3508_2, a2);
    Class_Motor_C620_Set_Target_Angle(&motor_3508_3, a3);
    Class_Motor_C620_Set_Target_Angle(&motor_3508_4, a4);
  //Class_Motor_C620_Set_Target_Angle(&motor_3508_1, w1);
}

void Chassis_3508_PID_Run(void)
{
    Class_Motor_C620_TIM_PID_PeriodElapsedCallback(&motor_3508_1);
    Class_Motor_C620_TIM_PID_PeriodElapsedCallback(&motor_3508_2);
    Class_Motor_C620_TIM_PID_PeriodElapsedCallback(&motor_3508_3);
    Class_Motor_C620_TIM_PID_PeriodElapsedCallback(&motor_3508_4);

//    Chassis_Friction_FF(&motor_3508_1);
//    Chassis_Friction_FF(&motor_3508_2);
//    Chassis_Friction_FF(&motor_3508_3);
//    Chassis_Friction_FF(&motor_3508_4);
}
/* ==================== 夹爪控制实现 ==================== */

/**
 * @brief  夹爪 3508 初始化
 * @note   只初始化 PID 和 C620 类；“全开角”在第一帧 0x205 反馈
 *         到达后由 Gripper_Task() 记录，避免上电无反馈时误判。
 */
void Gripper_3508_Init(void)
{
    fp32 omega_pid[3] = {2500.0f, 0.15f, 0.01f};
    fp32 angle_pid[3] = {6.0f, 0.01f, 0.0f};

    /* 速度环：角速度 rad/s -> 电流 */
    PID_init(&motor_gripper.PID_Omega, PID_POSITION, omega_pid, 6000.0f, 4000.0f);

    /* 角度环：角度 rad -> 目标角速度 rad/s */
    PID_init(&motor_gripper.PID_Angle, PID_POSITION, angle_pid, 3.0f, 3.0f);

    /* ID5：反馈 0x205，控制帧 0x1FF（框架 allocate_tx_data 已支持） */
    Class_Motor_C620_Init(&motor_gripper, &hcan1, CAN_Motor_ID_0x205,
                          Control_Method_OMEGA, 19.0f, 16384.0f);

    Class_Motor_C620_Set_Target_Omega(&motor_gripper, 0.0f);

    g_grip_state      = GRIP_IDLE;
    g_grip_open_valid = 0;
    g_grip_stall_tick = 0;
}

/**
 * @brief  夹爪 CAN 接收回调（0x205）
 * @param  Rx_Buffer CAN1 收到的原始帧
 */
void Gripper_CAN_RxCallback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    Class_Motor_C620_CAN_RxCpltCallback(&motor_gripper, Rx_Buffer->Data);
}

/**
 * @brief  夹爪周期任务（建议 10ms 调用一次）
 * @note   必须在 TIM_CAN_PeriodElapsedCallback() 之前调用，
 *         这样本周期算出的电流才能在同一周期发到 0x1FF。
 */
void Gripper_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* 第一帧反馈到达后，把“手动全开”位置记为全开角 */
    if (!g_grip_open_valid && motor_gripper.Flag > 0)
    {
        g_grip_open_angle = motor_gripper.Now_Angle;
        g_grip_open_valid = 1;
    }

    /* 还没有收到 0x205 反馈：不输出，防止盲目驱动 */
    if (!g_grip_open_valid)
    {
        Class_Motor_C620_Set_Target_Omega(&motor_gripper, 0.0f);
        Class_Motor_C620_TIM_PID_PeriodElapsedCallback(&motor_gripper);
        return;
    }

    if (gripper_cmd == 1)   /* 1 = 向内夹取 */
    {
        if (g_grip_state != GRIP_HOLD)
        {
            /* 首次进入闭合：切速度环，给一个低速向内的目标 */
            if (g_grip_state != GRIP_CLOSING)
            {
                Class_Motor_C620_Set_Control_Method(&motor_gripper, Control_Method_OMEGA);
                PID_clear(&motor_gripper.PID_Angle);
                PID_clear(&motor_gripper.PID_Omega);
                Class_Motor_C620_Set_Target_Omega(&motor_gripper, GRIP_CLOSE_OMEGA);
                g_grip_state      = GRIP_CLOSING;
                g_grip_stall_tick = 0;
            }

            /* 堵转判定：电流够大 + 转速接近 0 */
            if ((motor_gripper.Now_Torque > GRIP_STALL_TORQUE ||
                 motor_gripper.Now_Torque < -GRIP_STALL_TORQUE) &&
                (motor_gripper.Now_Omega < GRIP_STALL_OMEGA &&
                 motor_gripper.Now_Omega > -GRIP_STALL_OMEGA))
            {
                if (g_grip_stall_tick == 0)
                {
                    g_grip_stall_tick = now;
                }
                else if ((now - g_grip_stall_tick) >= GRIP_STALL_MS)
                {
                    /* 夹住：切力矩环，固定保持电流，避免继续往里顶 */
                    Class_Motor_C620_Set_Control_Method(&motor_gripper, Control_Method_TORQUE);
                    PID_clear(&motor_gripper.PID_Angle);
                    PID_clear(&motor_gripper.PID_Omega);
                    Class_Motor_C620_Set_Target_Torque(&motor_gripper, GRIP_HOLD_TORQUE);
                    g_grip_state = GRIP_HOLD;
                }
            }
            else
            {
                g_grip_stall_tick = 0;
            }
        }
    }
    else if (gripper_cmd == 0)   /* 0 = 松爪，回到上电全开位置 */
    {
        if (g_grip_state != GRIP_OPENING && g_grip_state != GRIP_IDLE)
        {
            Class_Motor_C620_Set_Control_Method(&motor_gripper, Control_Method_ANGLE);
            PID_clear(&motor_gripper.PID_Angle);
            PID_clear(&motor_gripper.PID_Omega);
            Class_Motor_C620_Set_Target_Angle(&motor_gripper, g_grip_open_angle);
            g_grip_state = GRIP_OPENING;
        }

        /* 到位后保持 ANGLE 模式，角度环会把它稳在全开角 */
        if ((motor_gripper.Now_Angle > (g_grip_open_angle - GRIP_OPEN_TOL)) &&
            (motor_gripper.Now_Angle < (g_grip_open_angle + GRIP_OPEN_TOL)))
        {
            g_grip_state = GRIP_IDLE;
        }
    }
    /* gripper_cmd == 0xFF：无新指令，保持当前状态 */

    Class_Motor_C620_TIM_PID_PeriodElapsedCallback(&motor_gripper);
}
void Mecanum_Inverse_Kinematics(Chassis_Motion_t *chassis)
{
    float wheel_line_speed[4] = {0};
    wheel_line_speed[0] = -chassis->vx -chassis->vy + L_PLUS_W * chassis->wz;
    wheel_line_speed[1] = -chassis->vx +chassis->vy + L_PLUS_W * chassis->wz;
    wheel_line_speed[2] =  chassis->vx +chassis->vy + L_PLUS_W * chassis->wz;
    wheel_line_speed[3] =  chassis->vx -chassis->vy + L_PLUS_W * chassis->wz;
    for(int i=0; i<4; i++)
    {
        chassis->motor_target_speed[i] = wheel_line_speed[i] ;
    }
//		chassis->motor_target_speed[0]=fmaxf(-3.0f, fminf(3.0f, chassis->motor_target_speed[0]));
//		chassis->motor_target_speed[1]=fmaxf(-3.0f, fminf(3.0f, chassis->motor_target_speed[1]));
//		chassis->motor_target_speed[2]=fmaxf(-3.0f, fminf(3.0f, chassis->motor_target_speed[2]));
//		chassis->motor_target_speed[3]=fmaxf(-3.0f, fminf(3.0f, chassis->motor_target_speed[3]));

    Chassis_3508_Set_Target_Omegas(chassis->motor_target_speed[0],chassis->motor_target_speed[1],chassis->motor_target_speed[2],chassis->motor_target_speed[3]);
}
void Chassis_Param_Init(Chassis_Motion_t *chassis)
{   
    chassis->vx = 0.0f;
    chassis->vy = 0.0f;
    chassis->wz = 0.0f;
    for(int i=0; i<4; i++) chassis->motor_target_speed[i] = 0.0f;
}

void Chassis_3508_Get_VOFA_Data(float *buf)
{
    buf[0] = motor_3508_2.Target_Omega;
    buf[1] = motor_3508_2.Now_Omega;
    buf[2] = motor_3508_2.Now_Torque;
    buf[3] = motor_3508_2.Out;
}

// void up_task(void){
//     if(up_flag){ 
//         //前轮抬升
//         HAL_Delay(1000); 
//         HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);   /* 迷你气缸伸出 */
//         //后轮向前
//         HAL_Delay(3000);
//         HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET); /* 迷你气缸缩回 */
//         //后轮升起
//         HAL_Delay(2000);
//         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   /* 迷你气缸伸出 */
//         HAL_Delay(1000);
//         //前轮收回
//         up_flag=0;
//     }
//     if(!up_flag){
//         //前轮向前
//         HAL_Delay(4000);
//         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); /* 迷你气缸缩回 */
//         HAL_Delay(1000);
//         //后轮向前
//      }
//         HAL_Delay(4000);
// }

/* ==================== 航向保持 (IMU 辅助) ==================== */

/**
 * @brief 初始化航向保持控制器
 */
void ChassisHeading_Init(ChassisHeading_t *h)
{
    if (h == NULL) return;

    h->heading_error  = 0.0f;
    h->kp             = HEADING_KP_DEFAULT;
    h->ki             = HEADING_KI_DEFAULT;
    h->max_correction = HEADING_MAX_CORRECTION;
    h->correction     = 0.0f;
}

/**
 * @brief 清零累积航向误差 (阶段切换 / 启停时调用)
 */
void ChassisHeading_Reset(ChassisHeading_t *h)
{
    if (h == NULL) return;
    h->heading_error = 0.0f;
}

/**
 * @brief 航向保持更新 (每周期调用)
 *
 * P 项: 阻尼瞬时角速率 (检测到旋转就立即反向)
 * I 项: 修正累积航向漂移 (慢慢拉回原方向)
 * 死区: |gyro_z| < HEADING_DEADZONE 时忽略 (防止残余零偏累积)
 *
 * @param h       航向保持句柄
 * @param gyro_z  陀螺仪 Z 轴角速度 (rad/s), 直行时应为 0
 * @param dt      时间间隔 (秒)
 * @return wz 修正值 (rad/s), 叠加到运动学 wz 上
 */
float ChassisHeading_Update(ChassisHeading_t *h, float gyro_z, float dt)
{
    float correction;

    if (h == NULL) return 0.0f;

    /* 死区: 小于阈值视为噪声, 不累积也不响应 */
    if (gyro_z < HEADING_DEADZONE && gyro_z > -HEADING_DEADZONE)
    {
        gyro_z = 0.0f;
    }

    /* 累积航向误差: 直行时 gyro_z 应恒为 0 */
    h->heading_error += gyro_z * dt;

    /* 限幅累积误差, 防止积分饱和 */
    if (h->heading_error > 0.3f)
        h->heading_error = 0.3f;
    else if (h->heading_error < -0.3f)
        h->heading_error = -0.3f;

    /* P 阻尼 + I 修正 */
    correction = -h->kp * gyro_z - h->ki * h->heading_error;

    /* 限幅 */
    if (correction > h->max_correction)
        correction = h->max_correction;
    else if (correction < -h->max_correction)
        correction = -h->max_correction;

    h->correction = correction;
    return correction;
}

/* ==================== Yaw 闭环旋转控制器 ==================== */

#define YAW_TWO_PI 6.28318530717958647692f

static float Yaw_Normalize(float angle)
{
    while (angle > PI)
    {
        angle -= YAW_TWO_PI;
    }
    while (angle < -PI)
    {
        angle += YAW_TWO_PI;
    }
    return angle;
}

void YawController_Init(YawController_t *ctrl)
{
    if (ctrl == NULL) return;

    ctrl->state      = YAW_IDLE;
    ctrl->target_yaw = 0.0f;
    ctrl->kp         = YAW_KP_DEFAULT;
    ctrl->ki         = YAW_KI_DEFAULT;
    ctrl->i_max      = YAW_I_MAX;
    ctrl->error_i    = 0.0f;
    ctrl->max_wz     = YAW_MAX_WZ_DEFAULT;
    ctrl->threshold  = YAW_THRESHOLD_DEFAULT;
    ctrl->current_wz = 0.0f;
}

void YawController_StartRotation(YawController_t *ctrl, float current_yaw, float angle_delta)
{
    if (ctrl == NULL) return;

    ctrl->target_yaw = Yaw_Normalize(current_yaw + angle_delta);
    ctrl->state      = YAW_ROTATING;
    ctrl->current_wz = 0.0f;
    ctrl->error_i    = 0.0f;   /* 清零积分, 避免上次残留 */
}

bool YawController_Update(YawController_t *ctrl, float current_yaw)
{
    float error;
    float wz_p, wz_i;

    if (ctrl == NULL || ctrl->state != YAW_ROTATING)
    {
        return (ctrl != NULL && ctrl->state == YAW_DONE);
    }

    error = ctrl->target_yaw - current_yaw;
    error = Yaw_Normalize(error);

    /* 到位判断: 误差小于阈值 */
    if (error < ctrl->threshold && error > -ctrl->threshold)
    {
        ctrl->current_wz = 0.0f;
        ctrl->error_i    = 0.0f;
        ctrl->state = YAW_DONE;
        return true;
    }

    /* 积分累积 (T=0.01s, 梯形积分) */
    ctrl->error_i += error * 0.01f;
    if (ctrl->error_i >  ctrl->i_max) ctrl->error_i =  ctrl->i_max;
    if (ctrl->error_i < -ctrl->i_max) ctrl->error_i = -ctrl->i_max;

    /* PI 控制: wz = kp*error + ki*error_i */
    wz_p = ctrl->kp * error;
    wz_i = ctrl->ki * ctrl->error_i;
    ctrl->current_wz = wz_p + wz_i;

    /* 输出限幅 */
    if (ctrl->current_wz > ctrl->max_wz)
        ctrl->current_wz = ctrl->max_wz;
    else if (ctrl->current_wz < -ctrl->max_wz)
        ctrl->current_wz = -ctrl->max_wz;

    return false;
}

float YawController_GetWz(const YawController_t *ctrl)
{
    if (ctrl == NULL) return 0.0f;
    return ctrl->current_wz;
}

/* ==================== 底盘 90° 旋转封装 ==================== */

/**
 * @brief 初始化旋转模块: IMU 校准 + yaw 归零
 */
void ChassisRotate_Init(ChassisRotate_t *rt, MPU6500_t *imu)
{
    if (rt == NULL || imu == NULL) return;

    rt->state     = ROTATE_IDLE;
    rt->yaw       = 0.0f;
    rt->brake_dir = 0.0f;
    rt->brake_tick = 0;
    rt->imu_ready = true;

    YawController_Init(&rt->ctrl);

    /* 陀螺仪零偏校准 */
    MPU6500_CalibrateGyro(imu, 200);
}

/**
 * @brief 触发旋转指定角度
 *
 * @param rt      旋转句柄
 * @param degrees 角度 (度), +CW / -CCW, 范围 -360 ~ +360
 */
void ChassisRotate_ByAngle(ChassisRotate_t *rt, float degrees)
{
    float angle_rad;

    if (rt == NULL || rt->state != ROTATE_IDLE) return;

    /* 度转弧度, CW 为正 → yaw delta 为负 */
    angle_rad = -degrees * PI / 180.0f;

    rt->yaw = 0.0f;
    YawController_StartRotation(&rt->ctrl, 0.0f, angle_rad);

    /* 刹车方向: CW→正刹车, CCW→负刹车 */
    rt->brake_dir = (degrees >= 0.0f) ? 1.0f : -1.0f;
    rt->state = ROTATE_RUNNING;
}

/**
 * @brief 每周期调用, 执行完整旋转状态机
 * @return 当前应写入电机的 wz 值
 */
float ChassisRotate_Update(ChassisRotate_t *rt, MPU6500_t *imu)
{
    if (rt == NULL || imu == NULL || !rt->imu_ready) return 0.0f;

    /* 读取陀螺仪 + 积分 yaw */
    MPU6500_Read_Gyro(imu);
    MPU6500_Update_Yaw(imu, 0.01f);
    rt->yaw = imu->data.yaw;

    switch (rt->state)
    {
    case ROTATE_RUNNING:
    {
        YawController_Update(&rt->ctrl, rt->yaw);

        if (rt->ctrl.state == YAW_DONE)
        {
            /* 进入刹车阶段 */
            rt->state      = ROTATE_BRAKING;
            rt->brake_tick = 1;
        }
        else
        {
            return YawController_GetWz(&rt->ctrl);
        }
    }
    /* fall through */

    case ROTATE_BRAKING:
    {
        if (rt->brake_tick <= YAW_BRAKE_TICKS)
        {
            rt->brake_tick++;
            return YAW_BRAKE_SPEED * rt->brake_dir;
        }

        /* 刹车完成 → 清零 PID + 锁定 */
        PID_clear(&motor_3508_1.PID_Omega);
        PID_clear(&motor_3508_2.PID_Omega);
        PID_clear(&motor_3508_3.PID_Omega);
        PID_clear(&motor_3508_4.PID_Omega);
        motor_3508_1.Out = 0.0f;
        motor_3508_2.Out = 0.0f;
        motor_3508_3.Out = 0.0f;
        motor_3508_4.Out = 0.0f;

        rt->state = ROTATE_DONE;
        return 0.0f;
    }

    case ROTATE_DONE:
    case ROTATE_IDLE:
    default:
        return 0.0f;
    }
}
