<<<<<<< HEAD
=======
/*
Copyright (c) 2024 loopgad 9th_R2_Member

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
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
