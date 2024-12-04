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

#include "Peripheral_Control_Unit.h"

//使用xbox功能状态位
using namespace Xbox_Namespace;

Peripheral_Control::Peripheral_Control(UART_HandleTypeDef *action_huart_){
	action_huart = action_huart_;
}

//reset action module(with Action_Reset State switch) 默认用USART连接Action
inline void Peripheral_Control::Action_Reset(void) {
    if(Xbox_Namespace::Xbox_State_Info.Action_Reset){
        const char *str = "ACT0";
        while (*str) 
        {
            HAL_UART_Transmit(action_huart, (uint8_t *)str++, 1, HAL_MAX_DELAY);
        }
        Xbox_Namespace::Xbox_State_Info.Action_Reset = false; //清除状态位
    }
    
}


inline void Peripheral_Control::Air_Pump_Control(void){
    if(Xbox_Namespace::Xbox_State_Info.Air_Pump){
        
        Xbox_Namespace::Xbox_State_Info.Air_Pump = false; //清除状态位
    }
    else{
        
    }
}


void Peripheral_Control::Task_Function(void){
    Action_Reset();
}