/**
  ******************************************************************************
  * @file    moto.c
  * @author  陈斌
  * @version V1.1.0
  * @date    2023/3/29
  * @brief   
  ******************************************************************************
  */ 
#include "MoveBase.h"



/********************************************路径规划***************************************/
int testflag=0;//用于测试代码的进程


float kp_x = 6;
float kd_x = 0;	//0.00011
float kp_y = 6;
float kd_y = 0;	//0.00011
float kp_yaw = 1;
float kd_yaw = 0;
float error_X;float error_Y;	// 世界X、Y偏差
float error_x;float error_y;	// 本体x、y偏差
float error_Yaw;							// 偏航角偏差
float now_yaw;								// 当前弧度制偏航角
float u_output;								// 本体坐标x方向速度输出
float v_output;								// 本体坐标y方向速度输出
float w_ouput;								// 角速度输出
float Error=0;
ROBOT_CHASSIS Robot_Chassis;
/*pid相关结构体--------------------------------------*/
PID point_traker_x_pid;
PID point_traker_y_pid;
PID point_traker_yaw_pid;
//雷达追踪
PID point_traker_ladar_y_pid;
PID point_traker_ladar_x_pid;
PID yaw_pid_ladar;
PID catch_ring_pid;
/**
* @brief  移动相关数据初始化
* @note		
* @param  
* @retval 
*/
void Move_Init(void)
{
		//雷达底盘追踪
	PID_parameter_init(&point_traker_ladar_y_pid, 0.5,0.1, 0.1, 500, 0, 1);
	PID_parameter_init(&point_traker_ladar_x_pid, 0.5,0.1, 0.1, 500, 0, 1);
	PID_parameter_init(&yaw_pid_ladar, 30,0, 0.1, 800, 0, -1);

	//PD跟踪器
	PID_parameter_init(&point_traker_x_pid, 3,0, 0.5, 2000, 0, 10);
	PID_parameter_init(&point_traker_y_pid, 3,0, 0.5, 2000, 0, 10); 
	PID_parameter_init(&point_traker_yaw_pid, 30,0, 0.1,1000, 0, 1);
	
	// 激光PID
	PID_parameter_init(&laser_X_pid ,  1,0, 0.5, 500, 0, 10);
	PID_parameter_init(&laser_Y_pid ,  1,0, 0.5, 100, 0, 10);
	PID_parameter_init(&laser_K_pid ,  3,0, 0.3, 300, 0, 5);
	PID_parameter_init(&laser_B_pid ,  2,0, 0.5, 300, 0, 5);
	
	// 自动路径PID
	PID_parameter_init(&point_pid ,  1.2f,0, 0.5f, 500, 0, 5);
	PID_parameter_init(&yaw_pid ,  50.0f,0.1f, 1.0f, 1000.0f, 0.0f, 0.1f);
	PID_parameter_init(&catch_ring_pid ,  10.0f,0.0f, 1.0f, 100.0f, 0.0f, 1.0f);
}
/**
* @brief  PDController跟踪器
* @note		跟踪规划好的路径
* @param  target_point:单位时间要跟踪的点（需先规划好速度），robot_now_pos:机器人当前世界坐标下的位置
* @retval 
*/
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
	YawAdjust(target_point.Yaw);
	// 计算误差
	error_X = target_point.X - robot_now_pos.POS_X;
	error_Y = target_point.Y - robot_now_pos.POS_Y;
	//error_Yaw = target_point.Yaw - robot_now_pos.POS_YAW;
//	//角度制转换为弧度制
//	now_yaw = robot_now_pos.POS_YAW * PI / 180.0f;
//	// 换算到本体坐标
//	error_x =  cos(now_yaw) * error_X + sin(now_yaw) * error_Y;
//	error_y = -sin(now_yaw) * error_X + cos(now_yaw) * error_Y;
//	
//	// 计算速度
//	w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
//	u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
//																		target_point.V_y  * sin(now_yaw) + \
//																		w_ouput * error_y * cos(now_yaw) - \
//																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
//	v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
//																		target_point.V_y  * cos(now_yaw) - \
//																		w_ouput * error_y * sin(now_yaw) - \
//																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);
//																		 
//	// 换算为世界坐标系下的速度
//	Robot_Chassis.World_V[1] = -(u_output * cos(now_yaw) - v_output * sin(now_yaw));
//	Robot_Chassis.World_V[0] = -(u_output * sin(now_yaw) + v_output * cos(now_yaw));
//	Robot_Chassis.World_V[2]  = -w_ouput;
	PID_position_PID_calculation_by_error(&point_traker_x_pid, error_X);
	PID_position_PID_calculation_by_error(&point_traker_y_pid, error_Y);
	//PID_position_PID_calculation_by_error(&point_traker_yaw_pid, error_Yaw);
	
	//添加负号
	Robot_Chassis.World_V[1] = point_traker_x_pid.output;
	Robot_Chassis.World_V[0] = point_traker_y_pid.output;
	//Robot_Chassis.World_V[2]  = -point_traker_yaw_pid.output;
}

int PDControllertest(float x,float y,float w)
{
	int yawflag;
	yawflag=YawAdjust(w);
	// 计算误差
	error_X = x- ROBOT_REAL_POS_DATA.POS_X;
	error_Y = y - ROBOT_REAL_POS_DATA.POS_Y;
//	error_Yaw = w - ROBOT_REAL_POS_DATA.POS_YAW;

	PID_position_PID_calculation_by_error(&point_traker_x_pid, error_X);
	PID_position_PID_calculation_by_error(&point_traker_y_pid, error_Y);
//	PID_position_PID_calculation_by_error(&point_traker_yaw_pid, error_Yaw);
	
	//添加负号
	Robot_Chassis.World_V[1] = point_traker_x_pid.output;
	Robot_Chassis.World_V[0] = point_traker_y_pid.output;
	//Robot_Chassis.World_V[2]  = -point_traker_yaw_pid.output;
	
		if((ABS(ROBOT_REAL_POS_DATA.POS_X - x)<10.0f&&ABS(ROBOT_REAL_POS_DATA.POS_Y - y)<10.0f)&&yawflag==1)
	{
		return 1;
	}
		return 0;
}


int k;
float t;
float f1s;float f2s;float f3s;float f4s;
float last_X;float last_Y;float last_Yaw;
float Sx_error;float Sy_error;
float Hz;
int first_time_flag = 1;
PATH_TYPEDEF now_path_point;

/**
* @brief  PathPlan规划+跟踪
* @note		三次B样条规划，误差直接赋值，到达终点返回1，否则返回0
* @param  t_real:真实经过的时间，t_target:目标总时间，num:控制点数目+1，X、Y:控制点数组
* @retval 
*/
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{ 
	float PathPlanerror_X;
	float PathPlanerror_Y;
	k = (int)(t_real * num / t_target);	// 第k段
	t = t_real - k * t_target / num;		// 第k段时间
  t = t * num / t_target;							// 归一化

	// 位置样条函数
	f1s = (1 - t) * (1 - t) * (1 - t) / 6;
	f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
	f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
	f4s = (t * t * t) / 6;
	
	// 计算目标跟踪点
	now_path_point.X = X[k] * f1s + X[k+1] * f2s + X[k+2] * f3s + X[k+3] * f4s;
	now_path_point.Y = Y[k] * f1s + Y[k+1] * f2s + Y[k+2] * f3s + Y[k+3] * f4s;
	now_path_point.Yaw = Yaw[k] * f1s + Yaw[k+1] * f2s + Yaw[k+2] * f3s + Yaw[k+3] * f4s;
	if(first_time_flag)
	{
		now_path_point.V_x = 0;
		now_path_point.V_y = 0;
		now_path_point.W = 0;
		first_time_flag = 0;
		Hz = 1 / t_real;
	}
	else
	{
		now_path_point.V_x = (now_path_point.X - last_X) * Hz;
		now_path_point.V_y = (now_path_point.Y - last_Y) * Hz;
		now_path_point.W = (now_path_point.Yaw - last_Yaw) * Hz;
	}
	if(t_real < (t_target))
	{
	// PD跟踪器
	PDController(now_path_point, ROBOT_REAL_POS_DATA);
//	PathPlanerror_X=ABS(ROBOT_REAL_POS_DATA.POS_X-now_path_point.X);
//	PathPlanerror_Y=ABS(ROBOT_REAL_POS_DATA.POS_Y-now_path_point.Y);
	}	
	// 保留本次值
	last_X = now_path_point.X;
	last_Y = now_path_point.Y;
	last_Yaw = now_path_point.Yaw;
	
	// 到达终点
	if(t_real > (t_target))
	{
		
//		if(moving_point_track(X[num+3], Y[num+3], Yaw[num+3],200))
//		{
		first_time_flag = 1;
		Robot_Chassis.World_V[1] = 0;//x轴
		Robot_Chassis.World_V[0] = 0;//y轴
		
			return 1;
//		}
	
	} 
	else	return 0;
}

PID point_pid;//点对点追踪PID
PID yaw_pid;//角度PID

//点跟踪
float point_track_error_x;
float point_track_error_y;
int moving_point_track(float POS_X, float POS_Y, float POS_YAW,float V_max)
{
		YawAdjust(POS_YAW);
	 float error;
	
	  //计算误差
	
	error = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // 计算误差
	point_pid.outputmax = ABS(V_max);
  PID_position_PID_calculation_by_error(&point_pid, error);
	Robot_Chassis.World_V[1] =-(point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_X - POS_X) /error);//x轴
	Robot_Chassis.World_V[0] = -(point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_Y - POS_Y) /error);//y轴
	
	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<10&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y)<10)
	{
		return 1;
	}
		return 0;
}

/**
* @brief  YawAdjust偏航角控制
* @note		将偏航角控制在目标角度
* @param  Target_angle:要限制的值
* @retval 
*/
int YawAdjust(float Target_angle)
{
   float YawAdjust_error;
 
	 // 计算误差
   if(ROBOT_REAL_POS_DATA.POS_YAW*Target_angle >= 0)
   {
      YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
   }
   else
   {
		 if(ABS(ROBOT_REAL_POS_DATA.POS_YAW)+ABS(Target_angle) <= 180) YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
		 else 
		 {
				AngleLimit(&YawAdjust_error);
		 }
   }
   
   // 直接利用PID输出角速度
   PID_position_PID_calculation_by_error(&yaw_pid, YawAdjust_error);
  Robot_Chassis.World_V[2]= -yaw_pid.output;	// 底盘角速度 单位：rad/s
	 
	  if(ABS(YawAdjust_error)<0.5)return 1;
	 else 
	 {
		 return 0;
	 }
}	

/**
* @brief  AngleLimit角度限幅
* @note		将角度限制在-180°到180°
* @param  angle:要限制的值
* @retval 
*/

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}

///**
//* @brief  梯形规划
//* @note		将角度限制在-180°到180°
//* @param  
//* @retval 
//*/
//int TrapezoidPlaning_set(float real_time,float T,float POS_X,float POS_Y,float POS_YAW,float V_max,float V_ac,float V_de)
//{
//	YawAdjust(POS_YAW);
//	float error,error_0;
//	float t_1,t_2,t_3;
//	float target_POS,target_X,target_Y;
//	float ac_rate,de_rate;
//	//计算误差
//	error = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // 计算误差
//  //计算一些变量
//	ac_rate=V_max/(V_ac*T);
//	de_rate=V_max/(V_de*T);
//	
//	if(real_time/T<=V_ac)//加速段
//	{
//		target_POS = ac_rate*real_time*real_time/2.0f;
//	}
//	if((real_time/T>=V_ac)&&(real_time/T<=(1-V_de)))//匀速段
//	{
//		target_POS = ac_rate*(V_ac*T)*(V_ac*T)/2.0f+V_max*(real_time-V_ac*T);
//	}
//	if(real_time/T>(1-V_de))//减速段
//	{
//		target_POS = ac_rate*(V_ac*T)*(V_ac*T)/2.0f+V_max*(real_time-V_ac*T);
//	}
//	
//  //分解速度
//	target_X =(ROBOT_REAL_POS_DATA.POS_X - POS_X) /error;//x轴
//	target_Y =(ROBOT_REAL_POS_DATA.POS_Y - POS_Y) /error;//y轴
//	
//	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<10&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y)<10)
//	{
//		
//		return 1;
//	}
//	else return 0;
//		
//}
//void TrapezoidPlaning(float t_real)

/**
* @brief  梯形规划
* @note		将角度限制在-180°到180°
* @param  
* @retval 
*/
//int TrapezoidPlaning(float real_time,
//										 float T,
//										 float POS_X,float POS_Y,float POS_YAW,
//										 float V_start,
//										 float V_end,
//										 float V_max,
//										 float R_ac,
//										 float R_de,
//										 int* first_time_flag)
//{	
////	YawAdjust(POS_YAW);
//	float error,error_0;
//	float target_POS,target_X,target_Y;

//	
//	float Ssu;   //总路程
//	float Sac;   //加速路程
//	float Sde;   //减速路程
//	float Sco;   //匀速路程
//	float Aac;   //加速加速度
//	float Ade;   //减速加速度
//	float S;     //当前路程
//	float output_V;//输出的速度
//	float real_error;//真实误差
//	
//  //计算一些变量
//	if(*first_time_flag)//第一次
//	{
//		error_0=sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // 计算误差
//		*first_time_flag=0;
//	}

//	////	// 如果所配数据有误，则不执行速度规划		
//	if((R_ac > 1) || (R_ac < 0) ||		//加速路程的比例
//		 (R_de > 1) || (R_de < 0) ||	//减速路程的比例
//		 (V_max < V_start) )			//最大的速度<开始的速度 
//	{
//		Robot_Chassis.World_V[1]=0;  // 不运动
//		Robot_Chassis.World_V[0]=0;
//		return 1;
//	}
//		// 计算一些变量
//	Ssu = ABS(error_0); 	//总路程   
//	Sac = Ssu * R_ac;		//加速路程 =	总路程 * 加速路程的比例
//	Sde = Ssu * R_de;		//减速路程 =	总路程 * 减速路程的比例
//	Sco = Ssu - Sac - Sde;		//匀速路程 = 总路程 - 加速路程 - 减速路程
//	Aac = (V_max * V_max - V_start * V_start) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
//	Ade = (V_end * V_end - V_max *   V_max) / (2.0f * Sde);	
//	real_error=sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));
//	// 过滤异常情况
//	if(error_0<real_error)		//当真实误差比最初误差还大时
//	{
//		output_V = V_start;	//TARGET_RPM = 开始的速度
//	}

//	else
//	{
//		S = ABS(error_0 - sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y)));      //开始位置
//		
//		// 规划RPM
//		if     (S < Sac)       output_V = sqrt(2.0f * Aac * S + V_start * V_start);               // 加速阶段
//		else if(S < (Sac+Sco)) output_V = V_max;                                                        // 匀速阶段
//		else                   output_V = sqrt(V_end * V_end - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
//	}
//	 
//  //分解速度，并分配合适得正负号
//	Robot_Chassis.World_V[1] =(output_V * 1.0f*(POS_X - ROBOT_REAL_POS_DATA.POS_X) /real_error);//x轴
//	Robot_Chassis.World_V[0] = (output_V * 1.0f*(POS_Y - ROBOT_REAL_POS_DATA.POS_Y) /real_error);//y轴
//	
//	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<3&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y)<3)
//	{
//		
//		return 1;
//		output_V=V_end;
//	}
//	else return 0;
//		
//}
//	

int chassis_TrapezoidPlaning(float POS_X_start,
	                    float POS_Y_start,
											float POS_X_end,
											float POS_Y_end,
											float POS_YAW,
											float V_start,
											float V_end,
											float V_max,
											float R_ac,
											float R_de)
{
//定义变量名称
	float Ssu_chassis;   //总路程
	float Sac_chassis;   //加速路程
	float Sde_chassis;   //减速路程
	float Sco_chassis;   //匀速路程
	float Aac_chassis;   //加速加速度
	float Ade_chassis;   //减速加速度
	float S_chassis;     //当前路程
	float output_V;//输出的速度
	float real_error;//真实误差
	float x_error_end_Trapezoid;//到结束点的误差
	float y_error_end_Trapezoid;
	float x_error_start_Trapezoid;//到开始点的误差
	float y_error_start_Trapezoid;

	//怀疑这里出了问题，可能需要添加上面两个变量来改进
	//YawAdjust(POS_YAW);
		// 如果所配数据有误，则不执行速度规划		
	if((R_ac > 1) || (R_ac < 0) ||		//加速路程的比例
		 (R_de > 1) || (R_de < 0) ||	//减速路程的比例
		 (V_max < V_start) )			//最大的速度<开始的速度 
	{
		Robot_Chassis.World_V[1]=0;  // 不运动
		Robot_Chassis.World_V[0]=0;
		return 1;
	}
	
	//计算行程变量
	 x_error_end_Trapezoid=POS_X_end - ROBOT_REAL_POS_DATA.POS_X;//到结束点的误差
	 y_error_end_Trapezoid=POS_Y_end - ROBOT_REAL_POS_DATA.POS_Y;
	 x_error_start_Trapezoid=POS_X_start - ROBOT_REAL_POS_DATA.POS_X;//到开始点的误差
	 y_error_start_Trapezoid=POS_Y_start - ROBOT_REAL_POS_DATA.POS_Y;
	
	Ssu_chassis=sqrt((POS_X_end-POS_X_start)*(POS_X_end-POS_X_start)+(POS_Y_end-POS_Y_start)*(POS_Y_end-POS_Y_start));
	if(Ssu_chassis<8500&&V_max>3400)
	{V_max=3400;}
	else if (Ssu_chassis<4000&&V_max>3000)
		V_max=1500;
	
	Sac_chassis=Ssu_chassis*R_ac;
	Sde_chassis=Ssu_chassis*R_de;
	Sco_chassis=Ssu_chassis-Sac_chassis-Sde_chassis;
	Aac_chassis = (V_max * V_max - V_start * V_start) / (2.0f * Sac_chassis);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
//  	if(Aac_chassis>1800)
//		Aac_chassis=1200;//500mm/s
	Ade_chassis = (V_end * V_end - V_max *   V_max) / (2.0f * Sde_chassis);	//减速加速度
//	  if(Ade_chassis>600)
//		Ade_chassis=600;//500mm/s
	
	real_error=sqrt((x_error_end_Trapezoid) * (x_error_end_Trapezoid) + (y_error_end_Trapezoid) * (y_error_end_Trapezoid));
		//过滤异常情况

		if(Ssu_chassis<S_chassis)
		{
		output_V = -V_start;	//TARGET_RPM = 开始的速度
		}
		
			else
	{
		S_chassis = sqrt((x_error_start_Trapezoid) * (x_error_start_Trapezoid) + (y_error_start_Trapezoid) * (y_error_start_Trapezoid));   //开始位置
		
		// 规划RPM
		if(S_chassis < Sac_chassis)      
			{
			output_V = sqrt(2.0f * Aac_chassis * S_chassis + V_start * V_start);
			//YawAdjust(POS_YAW);//加速阶段不调整角度

			}               // 加速阶段
		else if(S_chassis < (Sac_chassis+Sco_chassis)) 
			{	
			output_V = sqrt(2.0f * Aac_chassis * Sac_chassis + V_start * V_start);
			YawAdjust(POS_YAW);
			}			// 匀速阶段
		else if(S_chassis<Ssu_chassis)  
		{			
			output_V = sqrt(V_end * V_end - 2.0f * Ade_chassis * ABS(Ssu_chassis - S_chassis));
			YawAdjust(POS_YAW);
		}			// 减速阶段
		
		else 
			output_V=V_end;
			
	}
	
	//分解速度，并分配合适得正负号
	Robot_Chassis.World_V[1] =(output_V * 1.0f*(x_error_end_Trapezoid) /real_error);//x轴
	Robot_Chassis.World_V[0] = (output_V * 1.0f*(y_error_end_Trapezoid) /real_error);//y轴
	
	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X_end)<100&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y_end)<100)//提前跳出
	{
		testflag=1;
		output_V=0;
		Robot_Chassis.World_V[1]=0;
		Robot_Chassis.World_V[0]=0;
		return 1;
		
	}
	else return 0;
	
	
}



//特殊点跟踪
int near_pillar(float x,float y,float POS_YAW,float V_max)
{
	  
	 //计算误差
	 
		  if(ABS(y)>5||ABS(x)>5||ABS(POS_YAW)>0.1)
			{
   PID_position_PID_calculation_by_error(&point_traker_ladar_y_pid,y);
	 PID_position_PID_calculation_by_error(&point_traker_ladar_x_pid,x);
	 PID_position_PID_calculation_by_error(&yaw_pid_ladar,POS_YAW);
	 point_traker_ladar_y_pid.outputmax = ABS(V_max);
	 point_traker_ladar_x_pid.outputmax=ABS(V_max);
	 
	
	Robot_Chassis.World_V[0]=point_traker_ladar_y_pid.output;
	Robot_Chassis.World_V[1]=point_traker_ladar_x_pid.output;
	Robot_Chassis.World_V[2]=-yaw_pid_ladar.output;
			}
			else return 1;
			
	
		
		
		return 0;
		
}


  float S;     //当前路程
int	laser_speed_case=0;
int laser_speed_control(float Xstart,float Ystart,//开始时激光的数值
												float Xend,  float Yend,  //结束时目标的激光数值
												int Xnow,  int Ynow,	//实时的激光数值
												float Vmax,  float Vstart,float Vend,float POS_YAW)//最大速度，开始速度，结束速度
{
	YawAdjust(POS_YAW);
	  float Ssu;   //总路程
	  float Sac;   //加速路程
	  float Sde;   //减速路程
	  float Sco;   //匀速路程
	  float Aac;   //加速加速度
	  float Ade;   //减速加速度
	
	  
	float output;
	int laser_ok=0;
//	  float S_laser;
	
		Ssu = sqrt((Xstart-Xend)*(Xstart-Xend)-
							 (Ystart-Yend)*(Ystart-Yend));//总路程
		S = sqrt((Xnow-Xstart)*(Xnow-Xstart)-
						 (Ynow-Ystart)*(Ynow-Ystart));//当前路程
		Aac = 500;	//加速加速度 
		
	  if(Vstart > Vmax)   //启动速度快于最大速度 先降速
		{
			Aac = -ABS(Aac) ;	//加速度变减速 
		}
	  Ade = -500; //减速加速度
	  Sac = ( Vmax * Vmax - Vstart * Vstart ) / (2.0f * Aac);
	  Sde = ( Vend * Vend -  Vmax  *  Vmax  ) / (2.0f * Ade);
		Sco = Ssu - Sac - Sde ;//匀速路程
		
laser_speed_case=1;
	
		if((Sac + Sde ) > Ssu )//距离不够  计算出当前可以达到的最大速度Vmax 再根据Vmax重新分配路程
		{
			Aac = ABS(Aac);	
			Vmax = sqrt((2*Aac*Ade*Ssu+Ade*Vstart*Vstart-Aac*Vend*Vend)/(Ade-Aac));
		 if(Vstart > Vmax)    //启动速度快于最大速度  直接全程减速
		 {
			 Sde = Ssu;
			 Ade = ( Vend * Vend -  Vstart  *  Vstart ) / (2.0f * Sde);
			 Sco = 0;
			 Sac = 0;
laser_speed_case=2;
		 }
		 else  //加完速减速
		 {
			Sac = ( Vmax * Vmax - Vstart * Vstart ) / (2.0f * Aac) ;  //加速路程
	    Sde = ( Vend * Vend -  Vmax  *  Vmax  ) / (2.0f * Ade) ;  //减速路程
		  Sco = 0;
     laser_speed_case=3;
		 }
		}
   if     (S < Sac)            { output = sqrt(2.0f * Aac * S + Vstart * Vstart); laser_ok=0;}             	// 加速阶段
		else if(S < (Sac+Sco))      { output = Vmax; laser_ok=0;}                                                 	// 匀速阶段
		else if(S < (Sac+Sco+Sde))  { output = sqrt(Vend * Vend - 2.0f * Ade * ABS(Ssu - S)); laser_ok=0;}         // 减速阶段
		else                        { output = Vend; laser_ok=1;}                                                 // 停止阶段
		//注意xy的速度对应的xy
		Robot_Chassis.World_V[1] = -output*0.1f*(Xstart-Xnow)/ABS(Ssu-S);//x误差/总误差
		Robot_Chassis.World_V[0]=  -output*0.1f*(Ystart-Ynow)/ABS(Ssu-S);//y误差/总误差
		return laser_ok;
	
	}
		
//定义变量名称
	float Ssu_laser;   //总路程
	float Sac_laser;   //加速路程
	float Sde_laser;   //减速路程
	float Sco_laser;   //匀速路程
	float Aac_laser;   //加速加速度
	float Ade_laser;   //减速加速度
	float S_laser;     //当前路程
	float output_V_laser;//输出的速度
	float real_error_laser;//真实误差
int chassis_TrapezoidPlaning_laser(float POS_X_start,
	                    float POS_Y_start,
											float POS_X_end,
											float POS_Y_end,
											float POS_YAW,
											float V_start,
											float V_end,
											float V_max,
											float R_ac,
											float R_de,
												int POS_X_now,
                        int POS_Y_now)
{
	
	//角度矫正
	YawAdjust(POS_YAW);
	// 如果所配数据有误，则不执行速度规划	
	if((R_ac > 1) || (R_ac < 0) ||		//加速路程的比例
		 (R_de > 1) || (R_de < 0) ||	//减速路程的比例
		 (V_max < V_start) )			//最大的速度<开始的速度 
	{
		testflag=1;
		Robot_Chassis.World_V[1]=0;  // 不运动
		Robot_Chassis.World_V[0]=0;
		return 0;
	}
	//计算行程变量
	Ssu_laser=sqrt((POS_X_end-POS_X_start)*(POS_X_end-POS_X_start)+(POS_Y_end-POS_Y_start)*(POS_Y_end-POS_Y_start));
	Sac_laser=Ssu_laser*R_ac;
	Sde_laser=Ssu_laser*R_de;
	Sco_laser=Ssu_laser-Sac_laser-Sde_laser;
	Aac_laser = (V_max * V_max - V_start * V_start) / (2.0f * Sac_laser);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
//  	if(Aac_chassis>1800)
//		Aac_chassis=1200;//500mm/s
	Ade_laser = (V_end * V_end - V_max *   V_max) / (2.0f * Sde_laser);	//减速加速度
//	  if(Ade_chassis>600)
//		Ade_chassis=600;//500mm/s
	real_error_laser=sqrt((POS_X_now - POS_X_end) * (POS_X_now - POS_X_end) + (POS_Y_now - POS_Y_end) * (POS_Y_now - POS_Y_end));
	//过滤异常情况
	
		if(Ssu_laser<S_laser)
		{
		output_V_laser = V_start;	//TARGET_RPM = 开始的速度
		}
	
			else
	 {
		S_laser = sqrt((POS_X_now - POS_X_start) * (POS_X_now - POS_X_start) + (POS_Y_now - POS_Y_start) * (POS_Y_now- POS_Y_start));   //开始位置
		
		// 规划RPM
		if     (S_laser < Sac_laser)       output_V_laser = sqrt(2.0f * Aac_laser * S_laser + V_start * V_start);               // 加速阶段
		else if(S_laser < (Sac_laser+Sco_laser)) output_V_laser = sqrt(2.0f * Aac_laser * Sac_laser + V_start * V_start);                                                        // 匀速阶段
		else if (S_laser<Ssu_laser)                 output_V_laser = sqrt(V_end * V_end - 2.0f * Ade_laser * ABS(Ssu_laser - S_laser));  // 减速阶段
 	  else                                     output_V_laser=V_end;
	 }
	 	//分解速度，并分配合适得正负号
	if(direction==2)//方向二，机器人往右边移动，k为x,b为y
	{Robot_Chassis.World_V[1] =-(output_V_laser * 1.0f*(POS_X_end - POS_X_now) /real_error_laser);//x轴
	Robot_Chassis.World_V[0] = (output_V_laser * 1.0f*(POS_Y_end - POS_Y_now) /real_error_laser);//y轴
	testflag=0;
	}
	else if(direction==1)//或者，机器人往左边移动，b或x为x,k或y 为y
	{
	Robot_Chassis.World_V[1] =(output_V_laser * 1.0f*(POS_X_end - POS_X_now) /real_error_laser);//x轴
	Robot_Chassis.World_V[0] = (output_V_laser * 1.0f*(POS_Y_end - POS_Y_now) /real_error_laser);//y轴
	

	}
	else
	{
	Robot_Chassis.World_V[1] =(output_V_laser * 1.0f*(POS_X_end - POS_X_now) /real_error_laser);//x轴
	Robot_Chassis.World_V[0] = -(output_V_laser * 1.0f*(POS_Y_end - POS_Y_now) /real_error_laser);//y轴

	}
	if(ABS(POS_X_now - POS_X_end)<5&&ABS(POS_Y_now - POS_Y_end)<5)
	{
		testflag=1;
		 if(direction==1)
		 {
	Robot_Chassis.World_V[1] =-50;
	Robot_Chassis.World_V[0]=-50;
		 }
		 else if(direction==2)
		 {
		Robot_Chassis.World_V[1] =50;
	  Robot_Chassis.World_V[0]=-50;
		 }
		 else
			 output_V_laser=V_end;
		return 1;
		
	}
	else return 0;
}



////修改用于取环定位,末速度不为0
//int Laser_calibration_point(float k, float b,float yaw,float v_max,int direction_1)
//{
//	float error_K_point,error_B_point,ERROR_point;

//	YawAdjust(yaw);
//	//转到指定角度
//	error_K_point =  location_k - k;
//	error_B_point =  location_b - b;
//	ERROR_point=sqrt(error_K_point*error_K_point+error_B_point*error_B_point);
////判断距离是否合适
//	if(error_K_point<1500&&error_B_point<1500)
//	{
//		if(ABS(error_K_point)>8||ABS(error_K_point)>8)
//		{
////pid直接输出 
//			laser_K_pid.outputmax = ABS(v_max);
//			PID_position_PID_calculation_by_error(&laser_K_pid, ERROR_point);
//			if(direction_1==1)
//			{
//			Robot_Chassis.World_V[0] = -laser_K_pid.output*1.0f*error_K_point/ERROR_point;
//			Robot_Chassis.World_V[1]= -laser_K_pid.output*1.0f*error_B_point/ERROR_point;
//			}
//			if(direction_1==2)
//			{
//			Robot_Chassis.World_V[1] = laser_K_pid.output*1.0f*error_K_point/ERROR_point;
//			Robot_Chassis.World_V[0]= -laser_K_pid.output*1.0f*error_B_point/ERROR_point;
//			}
//			error_K_point =  location_k - k;
//			error_B_point =  location_b - k;
//		}
//		else
//			return 1;
//	}
//   
//	return 0;
//}
//	

