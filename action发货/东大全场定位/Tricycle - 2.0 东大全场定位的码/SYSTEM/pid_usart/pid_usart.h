#ifndef __PID_USART_H
#define __PID_USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
	
#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

void pid_uart_init(u32 bound);
void send_data();
#endif


