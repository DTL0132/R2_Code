#ifndef __SBUS_H
#define __SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

/* Exported variables --------------------------------------------------------*/
	extern uint16_t failsafe_status;
	extern uint16_t SBUS_footer;
	extern uint8_t  buf[25];
	extern uint16_t CH[18];
  extern uint8_t CH1[24];
	extern uint8_t  pre_catch_buf[1];
	extern uint8_t L;
	extern uint8_t R;
	extern int      pre_init_counter;
	extern bool     start_counting;
	extern bool     catch_signal;
	extern uint8_t i;

	/* Exported functions --------------------------------------------------------*/
	/* ���ڴ�����SBUS��ʼ���Ⱥ������� */
/* sbus.h ׷������ */
//typedef struct {
//    int16_t x;      // ����ƽ���ٶ� (-1000~1000)
//    int16_t y;      // ǰ���ٶ� (-1000~1000)
//    int16_t rotate; // ��ת�ٶ� (-1000~1000)
//} ChassisSpeed_t;

typedef struct {
    float vx;   
    float vy;
    float wz;
    float motor_target_speed[4];
} Chassis_Motion_t;
void SBUS_UpdateChassis(Chassis_Motion_t* chassis_motion);
void SBUS_RxCpltHandler(UART_HandleTypeDef* huart);
#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H */
