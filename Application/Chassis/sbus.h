#ifndef __SBUS_H
#define __SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported variables --------------------------------------------------------*/
	#define RX_BUF_SIZE 8       // 协议最大帧长
	extern uint16_t failsafe_status;
	extern uint16_t SBUS_footer;
	extern uint8_t  buf[25];
	extern uint16_t CH[18];
  extern uint8_t CH1[24];
	extern int8_t sign;
	extern uint8_t  pre_catch_buf[1];
	extern int      pre_init_counter;
	extern bool     start_counting;
	extern bool     catch_signal;
	extern uint8_t i;
	extern uint8_t rx_buf[RX_BUF_SIZE];    // DMA 接收缓冲区;
	/* Exported functions --------------------------------------------------------*/

typedef struct {
    float vx;   // 
    float vy;   // 
    float wz;   // 
    float motor_target_speed[4];
} Chassis_Motion_t;
void Chassis_Communication_Task(Chassis_Motion_t* chassis_motion);
//void UART_DMA_RX_Start(void);
//void UART_RX_Task(void);
#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H */
