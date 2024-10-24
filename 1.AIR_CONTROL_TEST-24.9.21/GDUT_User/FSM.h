#ifndef __FSM__H_
#define __FSM__H_
#include "stdint.h"
#include "stm32f4xx.h"
#include "can.h"
#include "HareWare.h"

#include "moto.h"
#include "calculation.h"
#include "PID.h"
#include "path.h"
#include "MIT.h"



typedef enum MOVE_STATE_ITEMS
{
	MOVE_OK,
	
	MOVE_FREE,
	
	MOVE_STOP,
	
	//取环点
	MOVE_1_LOAD_POINT,
	//取环点
	MOVE_2_LOAD_POINT,
	//射环1
	MOVE_1_SHOOT,
	
	// 射环2
	MOVE_2_SHOOT,
	//任意地方到射环
	MOVE_SHOOT,
	//启动区
	MOVE_1_RESTART,
	
	MOVE_LASER_NEAR,
	
	MOVE_LASER,
	//手动校准
	MOVE_MANUAL,
	//可调移动到取环点
		MOVE_LOAD,
	
}MOVE_STATE_ITEMS;


typedef enum SHOOTING_STATE_ITEMS
{

control_nineblock_0,//初始状态
	

control_nineblock_1,//射击近左1号柱

control_nineblock_2,//射击近中2号柱

control_nineblock_3,//射击近右3号柱
	
control_nineblock_4,//射击中左4号柱

control_nineblock_5,//停止射击

control_nineblock_6,//射击中右6号柱
	
control_nineblock_7,//远左柱子	

control_nineblock_8,//射击8最高号柱	

control_nineblock_9,//远右柱子
	
}SHOOTING_STATE_ITEMS;




extern MOVE_STATE_ITEMS MOVE_STATE;
extern SHOOTING_STATE_ITEMS SHOOTING_STATE;
extern int direction;

void FSM_Init(void);
void move_FSM(void);
void shoot_FSM(void);
void move(void);
void shoot(void);
void test(void);


#endif
