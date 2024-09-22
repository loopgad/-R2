/**
  ******************************************************************************
  * @file    robot.c
  * @author  梁立言
  * @version V1.0.0
  * @date    2023/5/17
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "moto.h"
#include "robot.h"
#include "cmsis_os.h"
#include "comunication.h"
#include "tim.h"
#include "calculation.h"
/* Private  variables ---------------------------------------------------------*/



//夹爪电机 ID:6
// arm电机 ID:5
/**********************
最上为0
最下为
home为150
*********************/

//1
//从中间下降
int Larm_down(void)
{
//	SHOOT_DATA.gun_state = 0;//不可以发射
//	if(MOTOR_REAL_INFO[5].CURRENT<10000&&KEY_DATA.KEY_armbottom==1){//按键检测
//	Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],60,250,100,1000,100,0.3,0.4);//下降规划，末速度不为0
//	return 0;
//	}
//	
//	else{
//	VelCrl(&MOTOR_REAL_INFO[5],0);	
//	MOTOR_REAL_INFO[5].REAL_ANGLE =265;
//	
//	return 1;
	
SHOOT_DATA.gun_state = 0;//不可以发射
	if((MOTOR_REAL_INFO[5].CURRENT>2000&&KEY_DATA.KEY_armbottom==0)||MOTOR_REAL_INFO[5].CURRENT>10000){//按键检测
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =265;
	return 1;	
	}
	else{
  Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],60,250,100,2000,500,0.3,0.4);//下降规划，末速度不为0
	return 0;
	}
}

//第一次下降
int Larm_down_first(void)
{

	
	
	SHOOT_DATA.gun_state = 0;//不可以发射
	if((MOTOR_REAL_INFO[5].CURRENT>2000&&KEY_DATA.KEY_armbottom==0)||MOTOR_REAL_INFO[5].CURRENT>10000){//按键检测
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =265;
	return 1;
	}
	
	else{
	
  Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],0,250,100,1500,500,0.3,0.4);//下降规划，末速度不为0
	return 0;
	}
}
//2
//等待校准
int Larm_wait(void)
{
VelCrl(&MOTOR_REAL_INFO[5],0);
	return 0;
}
//2
int Lclamp_close(void)
{
	static int cast_cnt5;
	Vel_TorqueCtrl(&MOTOR_REAL_INFO[6],10000,-4000);
	if(MOTOR_REAL_INFO[6].CURRENT<-6000)
		{		
    cast_cnt5++;			
							
		}
		else 
			cast_cnt5=0;
		if(cast_cnt5>50)
		{
			VelCrl(&MOTOR_REAL_INFO[6],0);
			MOTOR_REAL_INFO[6].vel_torquemode.flag = 0;
			return 1;			
		}
			else return 0;	
}

//3
int Larm_raise(void)
{

	static int cast_cnt4;
	if(MOTOR_REAL_INFO[5].CURRENT<-2000&&KEY_DATA.KEY_armtop==0){//按键检测
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =0;
		
	return 1;	
	}
	
	
	 if(MOTOR_REAL_INFO[5].CURRENT<-8000)
	 {
		 cast_cnt4++;
		 if(cast_cnt4>200)
		 {
			 VelCrl(&MOTOR_REAL_INFO[5],0);	
	     MOTOR_REAL_INFO[5].REAL_ANGLE =0;
	     return 1;	
		 }
	 }
		
	  if(ABS(MOTOR_REAL_INFO[5].CURRENT)<10000&&KEY_DATA.KEY_armtop==1)
			{
				cast_cnt4=0;
        Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],265,0,100,2000,100,0.4,0.4);//下降规划，末速度不为0	
				return 0;
			}
}     

//4
int Lclamp_open(void)
{
	
	Vel_TorqueCtrl(&MOTOR_REAL_INFO[6],5000,4000);
	if(MOTOR_REAL_INFO[6].CURRENT>4000)
	{
				CurrentCrl(&MOTOR_REAL_INFO[6],0);
				MOTOR_REAL_INFO[6].vel_torquemode.flag = 0;
		    MOTOR_REAL_INFO[5].velocity_planning.flag=0;
				MOTOR_REAL_INFO[6].REAL_ANGLE = 0;			
				return 1;
				}
	else return 0;
}

//新状态




//5
int Larm_down_to_home(void)
{
	Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],0,60,100,800,0,0.4,0.3);//下降规划，末速度不为0
	if(MOTOR_REAL_INFO[5].velocity_planning.flag == 1) 
	{	
	//VelCrl(&MOTOR_REAL_INFO[5],0);
	PosCtrl(&MOTOR_REAL_INFO[5],60);
	SHOOT_DATA.gun_state = 1;//可以发射
		MOTOR_REAL_INFO[5].velocity_planning.flag=0;
	return 1;
	}
	if((MOTOR_REAL_INFO[5].CURRENT>2000&&KEY_DATA.KEY_armbottom==0)||MOTOR_REAL_INFO[5].CURRENT>10000){//按键检测
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =265;
		MOTOR_REAL_INFO[5].velocity_planning.flag=0;
	return 1;
	}
	else return 0;
} 

//7
int Larm_raise_midle(void)
{
  Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],265,60,100,1500,0,0.4,0.4);//下降规划，末速度不为0	

if(MOTOR_REAL_INFO[5].velocity_planning.flag == 1) 
	{	
	//VelCrl(&MOTOR_REAL_INFO[5],0);
	PosCtrl(&MOTOR_REAL_INFO[5],60);
	
	return 1;
	}
	else return 0;
}

//8
int Larm_raise_end(void)
{
	SHOOT_DATA.yaw = 0;
				SHOOT_DATA.pitch = 0;		
static int cast_cnt4;
	if(MOTOR_REAL_INFO[5].CURRENT<-2000&&KEY_DATA.KEY_armtop==0){//按键检测
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =0;
	return 1;	
	}
	
	
	 if(MOTOR_REAL_INFO[5].CURRENT<-8000)
	 {
		 cast_cnt4++;
		 if(cast_cnt4>50)
		 {
			 VelCrl(&MOTOR_REAL_INFO[5],0);	
	     MOTOR_REAL_INFO[5].REAL_ANGLE =0;
	     return 1;	
		 }
	 }
		
	  if(ABS(MOTOR_REAL_INFO[5].CURRENT)<10000&&KEY_DATA.KEY_armtop==1)
			{
				cast_cnt4=0;
        Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],60,0,50,600,100,0.2,0.2);//下降规划，末速度不为0	
				return 0;
			}
}


//0
int Larm_home(void)
{
	//VelCrl(&MOTOR_REAL_INFO[5],0);
	//PosCtrl(&MOTOR_REAL_INFO[5],60);

	SHOOT_DATA.gun_state = 1;//可以发射
	return 0;
}


/**
  * @brief  推环机构控制
	* @param  None
	* @retval None
  * @attention
  */
void shoot_push(void)
{			
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 2000);
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
    osDelay(500);
		SHOOT_DATA.push_state = push;
}
void shoot_back(void)
{
		SHOOT_DATA.push_state = back;
		osDelay(500);
		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 2500);
}
	

//路径动作

