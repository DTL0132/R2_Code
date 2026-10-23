/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "sbus.h"
#include "bsp_can.h"
#include "pid.h"
#include "struct_typedef.h"
#include <stdio.h>
#include <string.h>
#include "gom_protocol.h"
#include "spi.h"
#include "imu_mpu6500.h"
#include "chassis_yaw_control.h"
#include "actuator_control.h"
#include "Unitree_control.h"
#include "chassis_control.h"
#include "chassis.h"
#include "drv_bsp.h"
#include "drv_can.h"
#include "dvc_motor.h"
#include "dvc_motor_dm.h"
#include "gpio.h"
#include "vofa_uart.h"
#include "actuator_control.h"
#include "chassis_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//pid_type_def chassis_pid[4];

uint32_t Counter = 0;
#define SAMPLE_FREQ 100.0f // 每秒采样 100 次
#define WAVE_FREQ 5.0f      // 波形频率 5Hz
/* 2π */
#define TWO_PI 6.28318530718f
float t = 0.0f; // 时间（秒）
const float dt = 1.0f / SAMPLE_FREQ;
int cbc=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern motor_measure_t motor_chassis[7];
extern MotorData_t data[4];
extern uint16_t CH[18];
extern uint8_t pre_catch_buf[1];
volatile uint8_t gripper_cmd = 0x1 ; // 夹爪指令0xFF = no command, 0 = open, 1 = close
volatile uint8_t cylinder_cmd_1 = 0x01; /* 前轮迷你气缸: 0=伸出, 1=收回, 0xFF=无命令 */
volatile uint8_t cylinder_cmd_2 = 0x01; /* 后轮迷你气缸: 0=伸出, 1=收回, 0xFF=无命令 */
volatile uint8_t suction_cmd  = 0x00; /* 吸盘:   0=释放, 1=吸取, 0xFF=无命令 */
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
//{
//    Actuator_CAN_RxCpltCallback(Rx_Buffer);
//    Chassis_3508_CAN_RxCallback(Rx_Buffer);
//}
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {

        case (0x01):
        {
            Class_Motor_DM_Normal_CAN_RxCpltCallback(&motor_4310_1, Rx_Buffer->Data);
        }
        break;
        case (0x201):
        {
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_1, Rx_Buffer->Data);
        }
        break;
        case (0x202):
        {
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_2, Rx_Buffer->Data);
        }
				break;
        case (0x203):
        {
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_3, Rx_Buffer->Data);
        }
        break;
        case (0x204):
        {
            Class_Motor_C620_CAN_RxCpltCallback(&motor_3508_4, Rx_Buffer->Data);
        }
        break;
        case (0x205):
        {
            /* 夹爪 3508：反馈数据交给夹爪电机实例 */
            Gripper_CAN_RxCallback(Rx_Buffer);
        }
        break;
    }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void chassis_task(void const * argument);
void motor_control_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    Actuator_Unitree_Init();
		// motion_control_init();
		// uint8_t ID[4], MODE[4];
		 HAL_UART_Receive_DMA(&huart7, pre_catch_buf, 1);
		// for (int z = 0; z < 4; z++)
		// {
		// ID[z] = z + 1;
		// MODE[z] = 1;
		// }
		// motor_select(ID,MODE);
//		  motor_mit_control(1, 0.06f, 0.03f, 0.10f, -10.0f, 0.1f);
//      motor_mit_control(2, 0.06f, 0.03f, 0.10f, -10.0f, 0.1f);
//  	  motion_control_run();
//		BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON, 0.0f, 0.0f);
    CAN_Init(&hcan1, CAN_Motor_Call_Back);

    /* 夹爪 3508（ID5）初始化：速度环 + 角度环 */
    Gripper_3508_Init();

//    // 3508电机初始化（PID参数配置 + 电机初始化 + 目标速度归零）
//    Chassis_3508_Omega_Init();
//    Class_Motor_DM_Normal_Init(&motor_4310_1, &hcan1, 0x01, 0x01, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA, 12.5f, 30.0f, 10.0f, 20.0f);
    Actuator_Init();
//		Actuator_Set_Control(0.0f,PI,0.0f,PI);
		Chassis_Motion_t chassis_motion;
		Chassis_Param_Init(&chassis_motion);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, chassis_task, osPriorityHigh, 0, 512);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, motor_control_task, osPriorityHigh, 0, 512);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
void chassis_task(void const * argument)
{
  /* USER CODE BEGIN chassis_task */
    // 3508电机初始化（PID参数配置 + 电机初始化 + 目标速度归零）
  // Chassis_3508_Omega_Init();
	Chassis_3508_Angle_Init();
	// Robo_chassis_init();
  for(;;)
  {
    
    Chassis_3508_Set_Target_Angles(PI/3, 0.0f, 0.0f, 0.0f);
    // Chassis_3508_Set_Target_Omegas(PI, 0.0f, 0.0f, 0.0f);
    Chassis_3508_PID_Run();

    /* 夹爪控制：闭合/保持/回开，必须在 CAN 发送前执行 */
    Gripper_Task();
    TIM_CAN_PeriodElapsedCallback();
			// Robo_chassis_task();
      osDelay(10);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_motor_control_task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_control_task */
void motor_control_task(void const * argument)
{
  /* USER CODE BEGIN motor_control_task */
//		Actuator_Set_Control(-2.0f,PI,0.0f,PI);
		osDelay(1);
  /* Infinite loop */
  for(;;)
  {

//			Actuator_Set_Control(-2.38f,PI,0.0f,PI);
//			Actuator_KeepAlive();   /* 内部每 100ms 检测一次 DM 电机存活 */
//			if(rx_buf[6]==1) Actuator_Set_Control(-2.7f,PI,0.0f,PI);
//			osDelay(1);
			if(cbc<=3000){
					motor_mit_control(1, 0.05f, 0.03f, 0.0f, 10.0f, 0.3f);
					cbc+=1;
			}else{
					motor_mit_control(1, 0.05f, 0.03f, 0.0f, 0.0f, 0.3f);
					cbc=0;
					osDelay(3000);
			}
  	  motion_control_run(); 
    osDelay(10);
  /* USER CODE END motor_control_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
}
