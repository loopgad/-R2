#include "point.h"
#include "PID.h"
#include "sys.h"
#include "pid_usart.h"

#define angle_stop_threshold  (0.1f)    //轨迹跟踪目标点角度停止阈值
#define pos_stop_threshold    (10.0f)   //轨迹跟踪目标点曼哈顿距离切换阈值

#define angle_swith_threshold  (1.0f)   //轨迹跟踪目标点角度切换阈值
#define pos_swith_threshold    (50.0f)  //轨迹跟踪目标点曼哈顿距离切换阈值

#define tracking_speed         500
#define points                 4
#define max_accel              200  //待修改成实际加速度
#define max_rotate_accel              300  //待修改成实际加速度
#define abs(x) ((x)>0? (x):(-(x)))




/*************设定模式***********/
//#define debug_single_point
//#define pid_update
//#define three_pid


typedef struct POINT{
    float x;
    float y;
    float z;
		char stop;
}POINT;

typedef struct GOAL_ERRORS{
    float x_error;
    float y_error;
    float xy_error;
		float z_error;
}GOAL_ERRORS;

struct LAST_SPEED{
    int Vx;
    int Vy;
    int Vz;
}last_speed;








extern float pos_x;
extern float pos_y;
extern float zangle;
extern float w_z;
extern int Vx,Vy,Vz;
extern u8 USART2_RX_STA;
extern float p1,i1,d1,p2,i2,d2,t1,t2;
extern float n1,n2;

PID pid_yaw , pid_x,pid_y,pid_position;

u8 init = 1;
u8 continue_run_flag = 1;

POINT point[points]={
	{ 200, 200, 10, 0},
	{ 250, 500, 20, 0},
	{ 250, 600, 30, 0},
	{ 0, 0, 0,  1}
};

GOAL_ERRORS goal_errors;
int count = 0;



void use_tracking()
{
	if(count < points)
	{
		track_point();
	}
}

/***********************************void speed_chabufa()*********************************/

void speed_restriction()
{
	if(Vx-last_speed.Vx > max_accel)  Vx = last_speed.Vx + max_accel;
	else if(Vx-last_speed.Vx < -max_accel)  Vx = last_speed.Vx - max_accel;
	if(Vy-last_speed.Vy > max_accel)  Vy = last_speed.Vy + max_accel;
	else if(Vy-last_speed.Vy < -max_accel)  Vy = last_speed.Vy - max_accel;	
	if(Vz-last_speed.Vz > max_rotate_accel)  Vz = last_speed.Vx + max_rotate_accel;
	else if(Vz-last_speed.Vz < -max_rotate_accel)  Vz = last_speed.Vz - max_rotate_accel;

	last_speed.Vx = Vx;
	last_speed.Vy = Vy;
	last_speed.Vz = Vz;
}




#ifndef three_pid
/*********************只在停止点使用pid，非停止点不断改变速度方向***************************/
void track_point()
{
	
	#ifdef pid_update
	if(USART2_RX_STA)
	{
		PIDInit(&pid_yaw , p1 , i1 , d1 , t2*30.0);
		point[count].z = t1;
		n1 = zangle;
		n2 = Vz;
		USART2_RX_STA = 0 ;
	}
	#endif
	
	
	/*************pid初始化**************/	
	if(init)
	{
		PIDInit(&pid_yaw, 50.0, 0.0, 20.0, 1000.0);
		pid_yaw.Derivative = 20.0;
		//		PIDInit(&pid_yaw, 50.0, 0.0, 2.0, 1000.0);pid_yaw.Derivative = 2.0;
		
		PIDInit(&pid_position , 20.0 , 0.0 , 0.0 , 1000.0);
		pid_position.Derivative = -10.0;
		init = 0 ;
	}
	
	/**********计算误差********/
	goal_errors.x_error = point[count].x - pos_x;
	goal_errors.y_error = point[count].y - pos_y;
	goal_errors.xy_error = abs(goal_errors.x_error) + abs(goal_errors.y_error);  //取曼哈顿距离的绝对值
	goal_errors.z_error = point[count].z - zangle;
	
	
	#ifndef debug_single_point
	/***********根据设定阀值切换当前点,当stop标志位为1时，停止增加，已完成当前任务，令continue_run_flag为1即继续轨迹跟踪任务*********/
	if( goal_errors.z_error < abs(angle_swith_threshold) )
	{
		if(goal_errors.xy_error < abs(pos_swith_threshold))
		{
			if( point[count].stop == 0 )
				count++;
			else if((point[count].stop == 1) && (continue_run_flag == 1))
			{
				//continue_run_flag = 0;
				count++;
			}
		}
	}
	#endif
	
	
	
	/***************姿态闭环******************/
	if( goal_errors.z_error<angle_stop_threshold && goal_errors.z_error>(-angle_stop_threshold) )
	{
    Vz = 0;
	}
	else
	{
		PID_Position_Calc(&pid_yaw, zangle, point[count].z);  //位置式pid
	  Vz = pid_yaw.output;
	}
	
	/****************路径跟踪*************/
	if(goal_errors.xy_error<pos_stop_threshold && goal_errors.xy_error>(-pos_stop_threshold))
	{
		Vx = 0;
		Vy = 0;
	}
	else
	{
		/*******若点为停止点，使用pid调速，若不是停止点，则只改变速度方向，不改变大小******/
		if(point[count].stop == 0)
		{
			Vx = tracking_speed * goal_errors.x_error / goal_errors.xy_error;
			Vy = tracking_speed * goal_errors.y_error / goal_errors.xy_error;
		}
		else
		{
			PID_Position_Calc(&pid_position, -goal_errors.xy_error, 0);
			Vx = pid_position.output* goal_errors.x_error/ goal_errors.xy_error;
			Vy = pid_position.output* goal_errors.y_error/ goal_errors.xy_error;
		}

	
	}
	
	/***********限定底盘运动最大加速度****************/
	//speed_restriction();

}	


#else
/***********点与点之间使用pid****************/
void track_point()
{
	#ifdef pid_update
	if(USART2_RX_STA)
	{
		PIDInit(&pid_yaw , p1 , i1 , d1 , t2*30.0);
		point[count].z = t1;
		n1 = zangle;
		n2 = Vz;
		USART2_RX_STA = 0 ;
	}
	#endif
	
	
	/*************pid初始化**************/	
	if(init)
	{
		PIDInit(&pid_yaw , 20.0 , 0.0 , 50.0 , 300.0);
//		PIDInit(&pid_position , 10.0 , 0.1 , 10 , 500.0);
		init = 0 ;
	}
	
	/**********计算误差********/
	goal_errors.x_error = point[count].x - pos_x;
	goal_errors.y_error = point[count].y - pos_y;
	goal_errors.xy_error = goal_errors.x_error + goal_errors.y_error;
	goal_errors.z_error = point[count].z - zangle;
	
	
	#ifndef debug_single_point
	/***********根据设定阀值切换当前点*********************/
	if(goal_errors.z_error<angle_stop_threshold && goal_errors.z_error>(-angle_stop_threshold))
	{
		if(goal_errors.xy_error<pos_stop_threshold && goal_errors.xy_error>(-pos_stop_threshold))
		{
			if( point[count].stop == 0 )
				count++;
			else if((point[count].stop == 1) && (continue_run_flag == 1))
			{
				continue_run_flag = 0;
				ount++;
			}
		}
	}
	#endif
	
	
	
	/***************姿态闭环******************/
	if( goal_errors.z_error<angle_stop_threshold && goal_errors.z_error>(-angle_stop_threshold) )
	{
    Vz = 0;
	}
	else
	{
		PID_Position_Calc(&pid_yaw, zangle, point[count].z);  //位置式pid
	  Vz = pid_yaw.output;
	}
	
	/****************路径跟踪*************/
	if(goal_errors.xy_error<pos_stop_threshold && goal_errors.xy_error>(-pos_stop_threshold))
	{
		Vx = 0;
		Vy = 0;
	}
	else
	{
		/*******若点为停止点，使用pid调速，若不是停止点，则只改变速度方向，不改变大小******/
			PID_Position_Calc(&pid_x, pos_x, point[count].x);
			PID_Position_Calc(&pid_y, pos_y, point[count].y);
			Vx = pid_x.output;
			Vy = pid_y.output;
	}
	
	//if()
	
	/***********限定底盘运动最大加速度****************/
	speed_restriction();
	
}
#endif
	