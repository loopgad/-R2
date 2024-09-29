#include"Base_Calculation.h"

/*	           
			   文件说明：
			   MOTOR_REAL_INFO来自moto
				ACTION_GL_POS_DATA与Robot_Chassis.Robot_V[y]来自Hardware->Action
	           这三个变量对应文件完成后再通过namesapce引用，记得改名字  
*/

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
void Calculation::Kinematic_Analysis_Inverse(void){

//	MOTOR_REAL_INFO[0].TARGET_RPM = 19*((0 + Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
//	MOTOR_REAL_INFO[1].TARGET_RPM = 19*((-(sqrt(3)/2) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
//	MOTOR_REAL_INFO[2].TARGET_RPM = 19*(((sqrt(3)/2) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);

	

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
void Calculation::Axis_analyse_for_WORLDtoROBOT(void)
{
//	/*改動了用於計算的角度值*/
//	// x方向上的转换
//	Robot_Chassis.Robot_V[x]= cos(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + sin(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[y]);
//	// y方向上的转换
//	Robot_Chassis.Robot_V[y]= -sin(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + cos(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180)*(Robot_Chassis.World_V[y]);
//	
//	// W方向上的转换
//	Robot_Chassis.Robot_V[w]= Robot_Chassis.World_V[w];

}

void Calculation::World_Control(void)        //始终以世界坐标下的Y方向为正Y
{
	static float KEEP_YAW=0;
	
//	Robot_Chassis.World_V[x]=-(ROCK_L_X-1500)*0.003f;
//	Robot_Chassis.World_V[y]=(ROCK_L_Y-1500)*0.003f;

	if(ROCK_R_X==1500)
	{
		if(ROCK_L_X == 1500 && ROCK_L_Y == 1500){
//			//Robot_Chassis.World_V[w];
//			Robot_Chassis.World_V[w] = 0;
//			
		}
//		KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;
//		//YawAdjust(KEEP_YAW);
	}
	else 
	{
//		Robot_Chassis.World_V[w]=(ROCK_R_X-1500)*0.01f;
		    //到时候会用消息队列传递，现在有缺陷
	}
	
	
}

void Calculation::Robot_Control(void)    //始终以机头方向为正Y
{
	
//	Robot_Chassis.Robot_V[x]=-(ROCK_L_X-1500)*0.03f;
//	Robot_Chassis.Robot_V[y]=(ROCK_L_Y-1500)*0.03f;
	if(ROCK_R_X==1500)
	{
//		Robot_Chassis.Robot_V[w]=(ROCK_R_X-1500)*0.01f;
//		KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;
	}
	else 
	{
//		YawAdjust(KEEP_YAW);
	}
	
}




