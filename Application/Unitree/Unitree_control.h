#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "main.h"
#include "gom_protocol.h"

// Motion control API
/**
  * @brief  Initializes the motion control module.
  * @param  None
  * @retval None
  */
void motion_control_init(void);

void motor_select(uint8_t* id,uint8_t* mode);

/**
  * @brief  Runs the main loop of the motion control.
  * @param  None
  * @retval None
  */
void motion_control_run(void);

// Parameter configuration API
/**
  * @brief  Sets the target torque for the motor.
  * @param  torque: The target torque value.
  * @retval None
  */
void set_motor_torque(int i, uint8_t id,float torque);

/**
  * @brief  Sets the target speed for the motor.
  * @param  speed: The target speed value.
  * @retval None
  */
void set_motor_speed(int i, uint8_t id,float speed);

/**
  * @brief  Sets the target position for the motor.
  * @param  position: The target position value.
  * @retval None
  */
void set_motor_position(uint8_t id, float position, float omega);

void set_motor_position_smoothly(uint8_t id, float target_position);
// Data feedback API
/**
  * @brief  Gets the feedback data from the motor.
  * @param  None
  * @retval const MotorData_t*: Pointer to the motor feedback data structure.
  */
const MotorData_t* get_motor_feedback(void);

void motor_mit_control(uint8_t id,float kp,float kw,float tor,float pos,float vel);

// 电机的零力矩模式
void motor_zero_control (uint8_t id);

#endif /* __MOTION_CONTROL_H */



