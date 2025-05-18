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

#include "Callback_Function.h"

#ifdef __cplusplus
extern "C" {
#endif

//串口缓冲区定义
uint8_t RxBuffer_for2[24] = {0};
uint8_t RxBuffer_for7[4] = {0};
uint8_t RxBuffer_for5[22] = {0};

/****************************启动串口接收中断******************************/
void USART_IT_Init(void){
    HAL_UART_Receive_IT(&huart2, RxBuffer_for2, 24);
	//HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart7, RxBuffer_for7, 4);
	//HAL_NVIC_EnableIRQ(UART7_IRQn);
    HAL_UART_Receive_IT(&huart5, RxBuffer_for5, 22);
  	//HAL_NVIC_EnableIRQ(UART5_IRQn);
}
/******************************************************************/


/***********************************************串口外设************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static Serialport_Drive my_serial;
	static uint8_t buffer_tmp[24] = {0}; //只用到前23



	// USART2 接收 Xbox 手柄信息
    if (huart == &huart2) {
		
        my_serial.Xbox_Receive_Data(RxBuffer_for2[0]);  // 处理接收的 Xbox 数据
		HAL_UART_Receive_IT(&huart2, RxBuffer_for2, 4);  // 重新启动中断接收
       
    }

    // 判断是否为USART5,接收action数据
    if (huart==&huart5) {
		
        my_serial.Action_Receive_Data(RxBuffer_for5); //接收并处理action数据
        //重新启动USART5接收中断
		HAL_UART_Receive_IT(&huart5,RxBuffer_for5, 22);
		
    }

		// 判断是否为USART7
    if (huart==&huart7) {

    	my_serial.GET_ROS_DATA(buffer_tmp);
		ROS_Namespace::Robot_Relative_Vx = my_serial.nextpoint[0];
		ROS_Namespace::Robot_Relative_Vy = my_serial.nextpoint[1];
		//重新启动USART7接收中断
		HAL_UART_Receive_IT(&huart2,RxBuffer_for2, 24);
	}
}

/***********************************************************************************/


#ifdef __cplusplus
}
#endif

