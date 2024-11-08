#include "Peripheral_Control_Unit.h"

//使用xbox功能状态位
using namespace Xbox_Namespace;

//reset action module(with Action_Reset State switch) 默认用USART连接Action
inline void Peripheral_Control::Action_Reset(void) {
    if(Xbox_State_Info.Action_Reset){
        const char *str = "ACT0";
        while (*str) 
        {
            HAL_UART_Transmit(&huart3, (uint8_t *)str++, 1, HAL_MAX_DELAY);
        }
        Xbox_State_Info.Action_Reset = false; //清除状态位
    }
    
}


inline void Peripheral_Control::Air_Pump_Control(void){
    if(Xbox_State_Info.Air_Pump){
        
        Xbox_State_Info.Air_Pump = false; //清除状态位
    }
    else{
        
    }
}


void Peripheral_Control::Task_Function(void){
    Action_Reset();
}