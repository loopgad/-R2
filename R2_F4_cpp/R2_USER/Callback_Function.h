#ifndef CALLBACK_FUNCTION_H
#define CALLBACK_FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "crc_util.h"
#include "Serialport_Drive.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>


extern UART_HandleTypeDef huart1; // 手柄使用
extern UART_HandleTypeDef huart2; // ROS 使用
extern UART_HandleTypeDef huart3; // action 使用

void USART_IT_Init(void);


#ifdef __cplusplus
}
#endif

#endif // CALLBACK_FUNCTION_H
