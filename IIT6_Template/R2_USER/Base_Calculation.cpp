#include"Base_Calculation.h"


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
inline void Calculation::Kinematic_Analysis_Inverse(void){
	//直接用浮点数代替sqrt计算，节省一点算力
	MOTOR_REAL_INFO[0].TARGET_RPM = 19*((0 + Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
	MOTOR_REAL_INFO[1].TARGET_RPM = 19*((-(0.866025403) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
	MOTOR_REAL_INFO[2].TARGET_RPM = 19*(((0.866025403) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);	
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
inline void Calculation::Axis_analyse_for_WORLDtoROBOT(void)
{
	/*改動了用於計算的角度值*/
	// x方向上的转换
	Robot_Chassis.Robot_V[x]= cos(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + sin(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[y]);
	// y方向上的转换
	Robot_Chassis.Robot_V[y]= -sin(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + cos(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180)*(Robot_Chassis.World_V[y]);
	
	// W方向上的转换
	Robot_Chassis.Robot_V[w]= Robot_Chassis.World_V[w];

}

void Calculation::World_Control(void)        //始终以世界坐标下的Y方向为正Y
{
	static float KEEP_YAW=0;
	//处理手柄遥感数据
	Xbox_State_Info.joyHori_LX = Xbox_State_Info.joyHori_LX > 31000 && Xbox_State_Info.joyHori_LX < 35000 ? 0 : Xbox_State_Info.joyHori_LX ;
	Xbox_State_Info.joyVert_LY = Xbox_State_Info.joyVert_LY > 31000 && Xbox_State_Info.joyVert_LY < 35000 ? 0 : Xbox_State_Info.joyVert_LY ; 
	Xbox_State_Info.joyHori_RX = Xbox_State_Info.joyHori_RX > 31000 && Xbox_State_Info.joyHori_RX < 35000 ? 0 : Xbox_State_Info.joyHori_RX ;
	//Xbox_State_Info.joyVert_RY = Xbox_State_Info.joyVert_RY > 31000 && Xbox_State_Info.joyVert_RY < 35000 ? 0 : Xbox_State_Info.joyVert_RY ; 
	//处理后的数据传入底盘解算
	Robot_Chassis.World_V[x] = Xbox_State_Info.joyHori_LX < 31000 ? ((Xbox_State_Info.joyHori_LX-31000)/31000) : ((Xbox_State_Info.joyHori_LX-35000)/35000);//换算百分比
	Robot_Chassis.World_V[y] = Xbox_State_Info.joyVert_LY < 31000 ? ((Xbox_State_Info.joyVert_LY-31000)/31000) : ((Xbox_State_Info.joyVert_LY-35000)/35000);//换算百分比

	//Robot_Chassis.World_V[x]=-(Xbox_State_Info.joyHori_LX-1500)*0.003f;
	//Robot_Chassis.World_V[y]=(Xbox_State_Info.joyVert_LY-1500)*0.003f;

	if(Xbox_State_Info.joyHori_RX==0 )
	{
		if(Xbox_State_Info.joyHori_LX == 0 && Xbox_State_Info.joyVert_LY == 0){
			//Robot_Chassis.World_V[w];
			Robot_Chassis.World_V[w] = 0;	
		}

		
		KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;
		//YawAdjust(KEEP_YAW);
	}
	else 
	{
		Robot_Chassis.World_V[w]=Xbox_State_Info.joyHori_RX < 31000 ? ((Xbox_State_Info.joyHori_RX-31000)/31000) : ((Xbox_State_Info.joyHori_RX-35000)/35000);//换算百分比

		    //到时候会用消息队列传递，现在有缺陷
	}
	//内联底盘解算
	Axis_analyse_for_WORLDtoROBOT();
	Kinematic_Analysis_Inverse();
}

void Calculation::Robot_Control(void)    //始终以机头方向为正Y
{
	//处理手柄遥感数据
	//处理手柄遥感数据
	Xbox_State_Info.joyHori_LX = Xbox_State_Info.joyHori_LX > 31000 && Xbox_State_Info.joyHori_LX < 35000 ? 0 : Xbox_State_Info.joyHori_LX ;
	Xbox_State_Info.joyVert_LY = Xbox_State_Info.joyVert_LY > 31000 && Xbox_State_Info.joyVert_LY < 35000 ? 0 : Xbox_State_Info.joyVert_LY ; 
	Xbox_State_Info.joyHori_RX = Xbox_State_Info.joyHori_RX > 31000 && Xbox_State_Info.joyHori_RX < 35000 ? 0 : Xbox_State_Info.joyHori_RX ;
	//Xbox_State_Info.joyVert_RY = Xbox_State_Info.joyVert_RY > 31000 && Xbox_State_Info.joyVert_RY < 35000 ? 0 : Xbox_State_Info.joyVert_RY ; 
	//数据传入解算
	Robot_Chassis.World_V[x] = Xbox_State_Info.joyHori_LX < 31000 ? ((Xbox_State_Info.joyHori_LX-31000)/31000) : ((Xbox_State_Info.joyHori_LX-35000)/35000);//换算百分比
	Robot_Chassis.World_V[y] = Xbox_State_Info.joyVert_LY < 31000 ? ((Xbox_State_Info.joyVert_LY-31000)/31000) : ((Xbox_State_Info.joyVert_LY-35000)/35000);//换算百分比

	//Robot_Chassis.Robot_V[x]=-(Xbox_State_Info.joyHori_LX-1500)*0.03f;
	//Robot_Chassis.Robot_V[y]=(Xbox_State_Info.joyVert_LY-1500)*0.03f;
	if(Xbox_State_Info.joyHori_RX==1500)
	{
		Robot_Chassis.Robot_V[w]=Xbox_State_Info.joyHori_RX < 31000 ? ((Xbox_State_Info.joyHori_RX-31000)/31000) : ((Xbox_State_Info.joyHori_RX-35000)/35000);//换算百分比
		//KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;
	}
	else 
	{
		//YawAdjust(KEEP_YAW);
	}
	//内联底盘解算
	Axis_analyse_for_WORLDtoROBOT();
	Kinematic_Analysis_Inverse();
}

//void Calculation::YawAdjust(float target_Yaw)
//{
//	
//}

void Calculation::Semi_auto_Control(void){

}

//根据速比进行速度映射
inline void Calculation::Speed_Level_Map(void){
	for(int i = 0 ; i < 3 ; i++){
		if(Xbox_State_Info.Speed_Threshold == 2){
			Robot_Chassis.Robot_V[i] = Robot_Chassis.Robot_V[i] * 1.3 ;
			Robot_Chassis.World_V[i] = Robot_Chassis.World_V[i] * 1.3 ;
		}
		else if(Xbox_State_Info.Speed_Threshold == 3){
			Robot_Chassis.Robot_V[i] = Robot_Chassis.Robot_V[i] * 1.8 ;
			Robot_Chassis.World_V[i] = Robot_Chassis.World_V[i] * 1.8 ;
		}
		else//速比为1则保持原值
		{	
		}
	}			
}


//读取手柄的底盘状态值进行模式选择
void Calculation::Move_State(void){

	Speed_Level_Map();//先根据手柄映射速度，然后再进行底盘解算

	switch(Xbox_State_Info.Base_Mode){
		case 1:
			Robot_Control();
		case 2:
			World_Control();
		case 3:
			Semi_auto_Control();
		default:
			Robot_Control();
	}

}

void Calculation::Task_Function(void){
	name = "Calculation_Task";
	Move_State();	

}


