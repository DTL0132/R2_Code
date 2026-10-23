/*changed code

* 编写MIT控制代码 void motor_mit_control(float kp,float kw,float tor,float pos,float vel)
* 编写电机选择代码 void motor_select(uint8_t id,uint8_t mode)
* 修改void set_motor_speed(float speed)
	void set_motor_position(float position)
  函数,将其输入值乘以电机传动减速比
* void motion_control_run(void)函数添加了错误处理,即将电机的mode置0(锁定模式)
*/

#include "Unitree_control.h"
#include "usart.h"
#include "PID.h"




// Private variables
HAL_StatusTypeDef tx_res[4];
HAL_StatusTypeDef rx_res[4];

MotorCmd_t cmd[4] = {0};
MotorData_t data[4] = {0};

/**
  * @brief  Runs the main loop of the motion control, called periodically.
  *         This function handles sending commands to the motor and receiving feedback.
  * @param  None
  * @retval None
  */
void motion_control_run(void)
{	
	for (int i = 0; i < 2; i++)
    {
		modify_data(&cmd[i]);
		
		tx_res[i] = HAL_UART_Transmit(&huart6, (uint8_t *)&cmd[i].motor_send_data, sizeof(cmd[i].motor_send_data),1);
		rx_res[i] = HAL_UART_Receive(&huart6, (uint8_t *)&data[i].motor_recv_data, sizeof(data[i].motor_recv_data),1);
		if (rx_res[i] == HAL_TIMEOUT)
		{
		  data[i].timeout++;
		}
		
		extract_data(&data[i]);
//		HAL_Delay(5);
	}
}


/**
  * @brief  Initializes the motion control module.
  * @param  None
  * @retval None
  */
void motion_control_init(void)
{
	for (int i = 0; i < 2; i++)
	{
		cmd[i].id = i+1;
		cmd[i].mode = 0;
		cmd[i].K_P = 0.0f;
		cmd[i].K_W = 0.0f;
		cmd[i].T = 0.0f;
		cmd[i].W = 0.0f;
		cmd[i].Pos = 0.0f; 
	}
}

void motor_select(uint8_t* id,uint8_t* mode)
{
	for (int i = 0; i < 2; i++)
	{
		cmd[i].id = id[i];
		cmd[i].mode = mode[i];
	}
}

/**
  * @brief  Sets the target torque for the motor.
  * @param  torque: The target torque value.
  * @retval None
  */
void set_motor_torque(int i, uint8_t id,float torque)
{
	cmd[i].id=id;
    cmd[i].T = torque;
}

/**
  * @brief  Sets the target speed for the motor.
  * @param  speed: The target speed value.
  * @retval None
  */
void set_motor_speed(int i, uint8_t id,float speed)
{
	cmd[i].id=id;
    cmd[i].W = speed * 6.33f;
	cmd[i].K_W=0.02f;
}

/**
  * @brief  Sets the target position for the motor.
  * @param  position: The target position value.
  * @retval None
  */
void set_motor_position(uint8_t id, float position, float omega)
{
	cmd[id-1].id=id;
	cmd[id-1].K_P=0.1f;
	cmd[id-1].K_W=0.05f;
  cmd[id-1].Pos = position; 
  cmd[id-1].W = omega * 6.33f; 
}

// 电机慢速运动到指定位置的位置模式
void set_motor_position_smoothly(uint8_t id, float target_position)
{
	float last_position;
	int i = id - 1;
	
	// 先读取电机当前数据
	extract_data(&data[i]);
		
	// 将当前电机的位置数据保存到last_position当中
	last_position = data[i].Pos;
	 
	// 将目标电机缓慢移动到指定位置（拆分成多目标）
	if (target_position > last_position) // 代表电机要正转
	{
		// 将目标电机缓慢移动到指定位置（拆分成多目标）
		for (float position = last_position; position <= target_position; position += 3)
		{
			if (target_position - position < 3)
				position = target_position;
			
			set_motor_position(id, position,0.5f);
			motion_control_run();
		}
	}
	else if (target_position < last_position) // 代表电机要反转
	{
		// 将目标电机缓慢移动到指定位置（拆分成多目标）
		for (float position = last_position; position >= target_position; position -= 3)
		{
			if (position - target_position < 3)
				position = target_position;
			
			set_motor_position(id, position, 0.5f);	
			motion_control_run();
		}
	}
	else // 目标位置就是当前位置
	{
		return;
	}
}

/**
  * @brief  Gets the feedback data from the motor.
  * @param  None
  * @retval const MotorData_t*: Pointer to the motor feedback data structure.
  */
/**
  * @brief  motor mit control
  * @param  kp,kw,tor,pos,vel
  * @retval None
  */

void motor_mit_control(uint8_t id,float kp,float kw,float tor,float pos,float vel)
{
	cmd[id-1].id=id;	
	float pos_des=pos;
	float vel_des=vel;
	
	float pos_error=pos_des - data[id-1].Pos;
	float vel_error=vel_des - data[id-1].W;
	float t_ref=tor+kp*pos_error+kw*vel_error;
	
	cmd[id-1].K_P=kp;
	cmd[id-1].K_W=kw;
	cmd[id-1].T=t_ref;
	cmd[id-1].Pos=pos_des;
	cmd[id-1].W=vel_des;
}

// 电机零力矩模式
void motor_zero_control (uint8_t id)
{
	cmd[id-1].id=id;
	cmd[id-1].K_P=0.0f;
	cmd[id-1].K_W=0.0f;
  cmd[id-1].Pos = 0.0f ;
	cmd[id-1].T = 0.0f;
  cmd[id-1].W = 0.0f;
}
//Unitree回调
// 回调里发下一包，实现不间断传输
