// sov.h
#ifndef __SOV_H
#define __SOV_H

#include "stm32f4xx_hal.h" 

// -------------------------- 气泵/电磁阀控制函数声明 --------------------------
void SolenoidValve_Trigger(uint32_t state); // 电磁阀单次触发（可自定义延时）

// -------------------------- 便捷调用宏 --------------------------
// 气泵控制宏
#define sov_open			 HAL_GPIO_WritePin(AIR_PUMP_PORT, AIR_PUMP_PIN, GPIO_PIN_SET)  // 开启气泵
#define sov_off      HAL_GPIO_WritePin(AIR_PUMP_PORT, AIR_PUMP_PIN, GPIO_PIN_RESET);  // 关闭气泵

// 电磁阀控制宏（默认1秒触发）
#define SolenoidValve_Trigger_On()  SolenoidValve_Trigger(1)
#define SolenoidValve_Trigger_Off()  SolenoidValve_Trigger(0)
#endif /* __SOV_H */
