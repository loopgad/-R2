#include "main.h"
#include "calculation.h"

/*!
 * \fn     Kinematic_Analysis
 * \brief  三全向轮运动逆解算
 *          直接设置目标速度值--->单位是米每秒
 * 			程序内部计算：米每秒--->转每分钟（轮子）--->转每分钟（转子）
 * \param  [in] float Vx   #
 * \param  [in] float Vy   #
 * 
 * \retval void
 */
void Kinematic_Analysis_Inverse(void)
{

	MOTOR_REAL_INFO[0].TARGET_RPM = 19*((0 + Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
	MOTOR_REAL_INFO[1].TARGET_RPM = 19*((-(number/2) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
	MOTOR_REAL_INFO[2].TARGET_RPM = 19*(((number/2) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
}
/*!
 * \fn     Axis_analyse_for_WORLDtoROBOT
 * \brief  完成世界坐标系下速度向机器人坐标系下的速度转
           换
 *          
 *		   用的是线速度
 * \param  [in] ROBOT_CHASSIS* ROBOT_NOW_INFO_IN_ITSELF   #
 * \param  [in] ROBOT_CHASSIS* ROBOT_NOW_INFO_IN_WORLD    #
 * 
 * \retval void
 */
void Axis_analyse_for_WORLDtoROBOT(void)
{
	// x方向上的转换
	Robot_Chassis.Robot_V[x]= cos(ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + sin(ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[y]);
	// y方向上的转换
	Robot_Chassis.Robot_V[y]= -sin(ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + cos(ACTION_GL_POS_DATA.REAL_YAW* PI / 180)*(Robot_Chassis.World_V[y]);
	// y方向上的转换
	Robot_Chassis.Robot_V[w]= Robot_Chassis.World_V[w];
	
}
float KEEP_YAW=0;
void World_Control(void)        //始终以世界坐标下的Y方向为正Y
{
	Robot_Chassis.World_V[x]=-(ROCK_L_X-1500)*0.003f;
	Robot_Chassis.World_V[y]=(ROCK_L_Y-1500)*0.003f;
	Robot_Chassis.World_V[w]=(ROCK_R_X-1500)*0.01f;
	if(ROCK_R_X==1500)
	{
		Robot_Chassis.World_V[w]=(ROCK_R_X-1500)*0.01f;
		KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;    //到时候会用消息队列传递，现在有缺陷
	}
	else 
	{
		YawAdjust(KEEP_YAW);
	}
	
	
}


void Robot_Control(void)    //始终以机头方向为正Y
{
	
	Robot_Chassis.Robot_V[x]=-(ROCK_L_X-1500)*0.03f;
	Robot_Chassis.Robot_V[y]=(ROCK_L_Y-1500)*0.03f;
	if(ROCK_R_X==1500)
	{
		Robot_Chassis.Robot_V[w]=(ROCK_R_X-1500)*0.01f;
		KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;
	}
	else 
	{
		YawAdjust(KEEP_YAW);
	}
	
	
	
}
void AUTO_Control(void)
{
	//自动路径规划，避障，敬请期待
}
void Contor_FSM(void)
{
	if(SWD==2000)   //拨杆D是开关
	{
		switch(SWB)  //拨杆B是模式控制
		{
			case 1000:
				Robot_Control();
				break;
			case 1500:
				World_Control();
			break;
			case 2000:
				AUTO_Control();
			break;
				
		}
				
				
	}
}