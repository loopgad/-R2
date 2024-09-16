#ifndef __ROBOT__H_
#define __ROBOT__H_
#include "stdint.h"
#include "stm32f4xx.h"
#include "can.h"
#include "HareWare.h"
#include "moto.h"
#include "calculation.h"
#include "PID.h"
#include "path.h"
#include "MIT.h"


/** 
  * @brief  存放关于整个机器人的标志位
  * @note     
  */
typedef struct ROBOT_FLAG_Type
{
	int move_ok; //0不允许移动；1允许移动
	int shoot_ok;//0不允许射击；1允许射击
	
//	int location;
	int Chassis_send;
	int Chassis_receive;
	int Gun_send;	//0装弹，1MOVE_1，2MOVE_2
	int Gun_receive;
}ROBOT_FLAG_Type;
/** 
  * @brief  左右发射机构
  * @note     
  */
typedef struct SHOOT_Type//发射数据
{
	float pitch;
	float yaw;
	float tiny_pitch;
	float tiny_pitch_data[9];
	float tiny_yaw;
	float tiny_yaw_data[9];
	float shoot_yaw;
	float shoot_pitch;
	float shout_speed_left;
	float shout_speed_right;
	int shout_ok;
	int push_state;
	int gun_state;//两个发射机构的状态0：初始位置（装弹） 1：手动发射  2：自动发射
}SHOOT_Type;

/** 
  * @brief  KEY
  * @note     
  */
typedef struct KEY_Type//发射数据
{
  int KEY_armtop;//armtop
	int KEY_armbottom;//armbottom
	int KEY_push;//push
	int KEY_left_clamp;
	int KEY_right_clamp;
}KEY_Type;


extern KEY_Type KEY_DATA;
extern ROBOT_FLAG_Type ROBOT_FLAG; 
extern SHOOT_Type SHOOT_DATA;//左边的发射机构
/* Exported functions ------------------------------------------------------- */
int CatchRing(void);
int Larm_down(void);
int Larm_down_first(void);
int Larm_wait(void);
int Lclamp_close(void);
int Larm_raise(void);
int Lclamp_open(void);
int Larm_down_to_home(void);
int Larm_home(void);
void shoot_push(void);
void shoot_back(void);

int Larm_raise_end(void);
int Larm_raise_midle(void);





#endif
