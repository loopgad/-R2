#ifndef __HARDWARE_PERIPHERAL_H
#define __HARDWARE_PERIPHERAL_H

#include "Callback_Function.h"

extern UART_HandleTypeDef huart3;


class Hardware_Peripheral{

    public:
        void Action_Reset(void);
        void Air_Pump_Control(uint16_t buf[10]);
};



#endif