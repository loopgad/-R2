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
#include "Peripheral_Control_Unit.h"

//使用xbox功能状态位
using namespace Xbox_Namespace;

//reset action module(with Action_Reset State switch) 默认用USART连接Action
inline void Peripheral_Control::Action_Reset(void) {
<<<<<<< HEAD
    if(Xbox_State_Info.Action_Reset){
=======
    if(Xbox_Namespace::Xbox_State_Info.Action_Reset){
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
        const char *str = "ACT0";
        while (*str) 
        {
            HAL_UART_Transmit(&huart3, (uint8_t *)str++, 1, HAL_MAX_DELAY);
        }
<<<<<<< HEAD
        Xbox_State_Info.Action_Reset = false; //清除状态位
=======
        Xbox_Namespace::Xbox_State_Info.Action_Reset = false; //清除状态位
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
    }
    
}


inline void Peripheral_Control::Air_Pump_Control(void){
<<<<<<< HEAD
    if(Xbox_State_Info.Air_Pump){
        
        Xbox_State_Info.Air_Pump = false; //清除状态位
=======
    if(Xbox_Namespace::Xbox_State_Info.Air_Pump){
        
        Xbox_Namespace::Xbox_State_Info.Air_Pump = false; //清除状态位
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
    }
    else{
        
    }
}


void Peripheral_Control::Task_Function(void){
    Action_Reset();
}