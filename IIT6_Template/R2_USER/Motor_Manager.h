#pragma once

#include "stm32h7xx_hal.h"
#include "Task_Manager.h"
#include "Global_Namespace.h"
#include "Motor.h"

using namespace Motor_Namespace;


class Motor_Manager : public Task_Thread //, public can 要加
{
private:
	
	void MotorrCtrl(void);
	void Motor_CAN_update(CAN_RxHeaderTypeDef *msg, uint8_t can1_RxData[8]);
	void MotorCURRENT_CAN_send(void);
		
public:
	void Motor_control(void);
	void Task_Function(void);
};

//用于创建电机类数组
void  Motor_Init(void);