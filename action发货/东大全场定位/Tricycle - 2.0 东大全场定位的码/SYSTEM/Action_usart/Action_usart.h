#ifndef __ACTION_USART_H
#define __ACTION_USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
	
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收

void Action_uart_init(u32 bound);

#endif


