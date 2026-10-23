#include "chassis_task.h"
#include "chassis.h"
#include "imu_mpu6500.h"
#include "chassis_yaw_control.h"
#include "spi.h"
#include "sbus.h"
#include "cmsis_os.h"
MPU6500_t         mpu6500;
ChassisRotate_t   rotate;
ChassisHeading_t  heading;
Chassis_Motion_t  motion;
float             wz,wz_correct;
uint8_t           chassis_phase=0;    //
uint8_t           up_flag=1;
uint8_t           up_reset = 0;       /* up_task 重置标志 */

/* ========== Phase 4 角度环状态变量 ========== */
static uint8_t  p4_entered     = 0;   /* 是否已进入 phase4 并完成 ANGLE 模式切换 */
static float    p4_yaw_ref     = 0.0f; /* 进入 phase4 时锁定的目标航向 (rad) */
static float    p4_angle_accum = 0.0f; /* 累计前进角度 (rad) */
#define 	front_extend            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);   	/* 前迷你气缸伸出 */
#define 	front_retract           HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET); 	/* 前迷你气缸缩回 */
#define 	middle_extend           HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);   	/* 中迷你气缸伸出 */
#define 	middle_retract          HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET); 	/* 中迷你气缸缩回 */
#define 	rear_extend				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   	/* 后迷你气缸伸出 */
#define     rear_retract 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);   /* 后迷你气缸伸出 */
void Robo_chassis_init(void){
    /* IMU初始化 */
    if(!MPU6500_Init(&mpu6500, &hspi5)){
        for(;;){
            Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
            Chassis_3508_PID_Run();
            TIM_CAN_PeriodElapsedCallback();
            osDelay(10);
        }
    }
    /* 旋转模块+航向保持初始化(含陀螺仪校准，约2秒，底盘保持静止！) */
    ChassisRotate_Init (&rotate,&mpu6500);
    ChassisHeading_Init(&heading);
    Chassis_Param_Init (&motion);
}
void Robo_chassis_task(void){
		if(rx_buf[3]==1){         /* 上位机触发旋转, rx_buf[3] 给度数 */
		    chassis_phase=5;
		}
//		if(rx_buf[4]==0&&rx_buf[3]==1){
//		    chassis_phase=2;
//		}
		if((rx_buf[1]!=0 || rx_buf[2]!=0 )&& chassis_phase!=1){
			chassis_phase=0;
		}
//		if(rx_buf[6]==1){         /* 上位机触发角度环直行 */
//		    chassis_phase=4;
//		}
		/* 相位切换检测: 退出 phase4 时恢复 OMEGA 模式 */
//		if (prev_chassis_phase == 4 && chassis_phase != 4)
//		{
//		    Chassis_3508_Omega_Init();
//		    p4_entered = 0;
//		}
//		prev_chassis_phase = chassis_phase;
//   
	switch (chassis_phase)
    {
        case 0:{
            /* ---- 前后左右 + 航向保持 ---- */
            /* 读取陀螺仪 */
            MPU6500_Read_Gyro(&mpu6500);

            /* 航向保持：直行时抑制左右偏移 */
            wz_correct = ChassisHeading_Update(&heading, mpu6500.data.gyro_z , 0.01f );

            /* 运动学：vx左右, vy前后, wz航向修正 */
            motion.vx = sign*rx_buf[2] / 2.0f;
						if(rx_buf[1]!=0)
            motion.vy = - sign * 3.8f;
            motion.wz = wz_correct;      // IMU航向修正
            Mecanum_Inverse_Kinematics(&motion);
        }
        break;
        case 1:{
            /* rx_buf[3] 给度数: +CW / -CCW, 范围 -128~+127° */
            if(rotate.state == ROTATE_IDLE)
            ChassisRotate_ByAngle(&rotate, rx_buf[3]*3.0f);

            wz =ChassisRotate_Update(&rotate, &mpu6500);
            Chassis_3508_Set_Target_Omegas(wz,wz,wz,wz);

            if(rotate.state == ROTATE_DONE)
            {
                ChassisHeading_Reset(&heading);  //切换阶段，清零累积误差
//								wz=rx_buf[3]*3.0f;
//                Chassis_3508_Set_Target_Omegas(wz,wz,wz,wz);
                Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);

            }
        }
        break;
        case 2:{
            /* rx_buf[3] 给度数: +CW / -CCW, 范围 -128~+127° */
            if(rotate.state == ROTATE_IDLE)
                ChassisRotate_ByAngle(&rotate, (float)((int8_t)rx_buf[3])*30.0f);

            wz =ChassisRotate_Update(&rotate, &mpu6500);
            Chassis_3508_Set_Target_Omegas(wz,wz,wz,wz);

            if(rotate.state == ROTATE_DONE)
            {
                ChassisHeading_Reset(&heading);  //切换阶段，清零累积误差
                Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);

            }
        }
        break;
        case 3:{   /* 越障任务 (非阻塞状态机) */
            static uint8_t prev_phase = 0;
            if (prev_phase != 3) {     /* 刚进入, 重置状态机 */
                up_task_reset();
            }
            prev_phase = 3;
            up_task();
        }
        break;
        case 4:{   /* 角度环直行 + 陀螺仪航向修正 */
            float yaw_error, heading_corr_angle;

            /* 首次进入: 切换到 ANGLE 模式 + 锁定当前航向 */
            if (!p4_entered)
            {
                Chassis_3508_Angle_Init();

                /* 清零角度环 PID 历史 */
                PID_clear(&motor_3508_1.PID_Angle);
                PID_clear(&motor_3508_2.PID_Angle);
                PID_clear(&motor_3508_3.PID_Angle);
                PID_clear(&motor_3508_4.PID_Angle);
                PID_clear(&motor_3508_1.PID_Omega);
                PID_clear(&motor_3508_2.PID_Omega);
                PID_clear(&motor_3508_3.PID_Omega);
                PID_clear(&motor_3508_4.PID_Omega);

                /* 锁定当前航向作为目标 */
                MPU6500_Read_Gyro(&mpu6500);
                MPU6500_Update_Yaw(&mpu6500, 0.0f);
                p4_yaw_ref     = mpu6500.data.yaw;
                p4_angle_accum = 0.0f;
                p4_entered     = 1;
            }

            /* 读取陀螺仪 + 更新 yaw */
            MPU6500_Read_Gyro(&mpu6500);
            MPU6500_Update_Yaw(&mpu6500, 0.01f);

            /* 航向误差 (归一化到 [-PI, PI]) */
            yaw_error = mpu6500.data.yaw - p4_yaw_ref;
            while (yaw_error > PI)  yaw_error -= 2.0f * PI;
            while (yaw_error < -PI) yaw_error += 2.0f * PI;

            /* 航向 P 修正 (角度偏移量) */
            heading_corr_angle = -1.5f * yaw_error;
            if      (heading_corr_angle >  1.0f) heading_corr_angle =  1.0f;
            else if (heading_corr_angle < -1.0f) heading_corr_angle = -1.0f;

            /* 前进/后退步进: rx_buf[1]前进, rx_buf[2]后退 */
            if (rx_buf[1] != 0)
            {
                p4_angle_accum += 4.0f * 0.01f;   /* 4 rad/s * 10ms = 0.04 rad/周期 */
            }
            else if (rx_buf[2] != 0)
            {
                p4_angle_accum -= 4.0f * 0.01f;
            }

            /* 角度目标分配 (麦轮正运动学反解):
             *   前进:  LF(-), RF(+), RR(+), LR(-)
             *   航向修正: 全部同向叠加 → 绕 Z 轴旋转
             */
            Chassis_3508_Set_Target_Angles(
                -p4_angle_accum + heading_corr_angle,   /* 电机1: 左前 LF */
                 p4_angle_accum + heading_corr_angle,   /* 电机2: 右前 RF */
                 p4_angle_accum + heading_corr_angle,   /* 电机3: 右后 RR */
                -p4_angle_accum + heading_corr_angle    /* 电机4: 左后 LR */
            );
        }
				case 5:{
							wz=sign*rx_buf[3]*2.5f;
              Chassis_3508_Set_Target_Omegas(wz,wz,wz,wz);
				}
        break;
    }

    /*3508 PID + CAN发送*/
    Chassis_3508_PID_Run();
    TIM_CAN_PeriodElapsedCallback();

}
/* 重置 up_task 状态机 (重新触发时调用) */
void up_task_reset(void){
    up_reset = 1;
}

/* up_task 状态机 (每 10ms 调用, 非阻塞) */
void up_task(void){
    static uint8_t  up_phase = 0;
    static uint32_t up_tick  = 0;

    if (up_reset) {
        up_phase = 0;
        up_tick  = 0;
        up_reset = 0;
    }

    switch (up_phase)
    {
    case 0:  /* 等待 1 秒 → 跳过, 直接转电机 */
      Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
      if (++up_tick > 100) { up_tick=0; up_phase=1; }
        up_tick=0; up_phase=1;
        break;
    case 1:  /* 前气缸伸出 + 后轮转 2.5 秒 */
        front_extend;
        Chassis_3508_Set_Target_Omegas(-5.0f,5.0f,0.0f,0.0f);
        if (++up_tick > 250) { up_tick=0; up_phase=2; }
        break;
    case 2:  /* 前气缸缩回 + 等 1 秒 */
        front_retract;
        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
        if (++up_tick > 200) { up_tick=0; up_phase=3; }
        break;
    case 3:  /* 中气缸伸出 + 前轮转 1 秒 */
        middle_extend;
        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,4.0f,-4.0f);
        if (++up_tick > 100) { up_tick=0; up_phase=4; }
        break;
    case 4:  /* 后气缸伸出 + 前轮转 1 秒 */
				rear_extend;
        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,4.0f,-4.0f);
        if (++up_tick > 100) { up_tick=0; up_phase=5; }
        break;
    case 5:  /* 中气缸缩回 + 等 1 秒 */
        middle_retract;
        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
        if (++up_tick > 150) { up_tick=0; up_phase=6; }
        break;
    case 6:  /* 后轮转 4 秒 */
        Chassis_3508_Set_Target_Omegas(-4.0f,4.0f,0.0f,0.0f);
        if (++up_tick > 100) { up_tick=0; up_phase=7; }
        break;
    case 7:  /* 后气缸收回*/
				rear_retract;
    default:
        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
        break;
    }
}

void down_task(void){
    static uint8_t  up_phase = 0;
    static uint32_t up_tick  = 0;

    switch (up_phase)
    {
    case 0:  /* 等待 1 秒 → 跳过, 直接转电机 */
//      Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
      if (++up_tick > 100) { up_tick=0; up_phase=1; }
        up_tick=0; up_phase=1;
        break;
    case 1:  /* 前气缸伸出 + 后轮转 3 秒 */
        rear_extend;
//        Chassis_3508_Set_Target_Omegas(-3.0f,3.0f,0.0f,0.0f);
        if (++up_tick > 100) { up_tick=0; up_phase=2; }
        break;
    case 2:  /* 前气缸缩回 + 等 2 秒 */
        middle_extend;
//        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
        if (++up_tick > 200) { up_tick=0; up_phase=3; }
        break;
    case 3:  /* 中气缸伸出 + 等 1 秒 */
        front_extend;
//        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
        if (++up_tick > 100) { up_tick=0; up_phase=4; }
        break;
    case 4:  /* 后气缸伸出+前轮转 4 秒 */
		rear_retract;
//        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,3.0f,-3.0f);
        if (++up_tick > 100) { up_tick=0; up_phase=5; }
        break;
    case 5:  /* 中气缸缩回 + 等 1 秒 */
        middle_retract;
//        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
        if (++up_tick > 150) { up_tick=0; up_phase=6; }
        break;
    case 6:  /* 后轮转 4 秒 */
//        Chassis_3508_Set_Target_Omegas(-3.0f,3.0f,0.0f,0.0f);
        if (++up_tick > 0) { up_tick=0; up_phase=7; }
        break;
    case 7:  /* 后气缸收回*/
		rear_retract;
    default:
//        Chassis_3508_Set_Target_Omegas(0.0f,0.0f,0.0f,0.0f);
        break;
    }
}
