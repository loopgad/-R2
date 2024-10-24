#pragma once 

#include "Global_Namespace.h"
#include "Task_Manager.h"

//使用xbox功能状态位
using namespace Xbox_Namespace;
//Action默认使用串口3
extern UART_HandleTypeDef huart3;

class Peripheral_Control : public Task_Thread
{
private:
    inline void Action_Reset(void);
    inline void Air_Pump_Control(void);
public:
    void Task_Function(void);
    
};

