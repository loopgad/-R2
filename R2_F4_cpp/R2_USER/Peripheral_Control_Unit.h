#pragma once 

#include "usart.h"
#include "Global_Namespace.h"



//Action默认使用串口3
extern UART_HandleTypeDef huart3;

class Peripheral_Control
{
private:
    inline void Action_Reset(void);
    inline void Air_Pump_Control(void);
public:
    void Task_Function(void);
    virtual ~Peripheral_Control() {}  // 添加虚析构函数
};

