#ifndef Motor_H
#define Motor_H

//#ifdef __cplusplus
//extern "C"
//{
//#endif
#include "stm32h7xx_hal.h"
#include "Global_Namespace.h"
#include "PID.h"
#include "Task_Manager.h"

//#ifdef __cplusplus
//}
//#endif

//#ifdef __cplusplus


using namespace Motor_Namespace;


// M3508电机编号
#define MotorR_ID_1      0x201
#define MotorR_ID_2      0x202
#define MotorR_ID_3      0x203


#define  SPEED_CONTROL_MODE				1		//速度模式


class Motor_Manager : public Task_Thread //, public can 要加
{
private:
	
	void MotorrCtrl(void);
	void Motor_CAN_update(CAN_RxHeaderTypeDef *msg, uint8_t can1_RxData[8]);
	void MotorCURRENT_CAN_send(void);
		
public:
	void Motor_control(void);
};

class Motor : public PID_Class 
{
private:
	PID_Class Motor_PID_RPM;//速度pid信息
	PID_Class Motor_PID_POS;//位置pid信息
		
public:
	friend void Motor_DeInit(Motor Motor[], , uint_fast8_t sizeof_Motor);
	friend Motor_Manager;
};



void  Motor_DeInit(void);


//#endif

#endif 