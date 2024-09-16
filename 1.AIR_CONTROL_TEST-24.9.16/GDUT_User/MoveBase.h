#ifndef __MOVEBASE_H
#define __MOVEBASE_H
/* Includes ------------------------------------------------------------------*/

#include "can.h"
#include "HareWare.h"

#include "moto.h"
#include "calculation.h"
#include "PID.h"
#include "stm32f4xx.h"
#include "FSM.h"
typedef struct
{
	float X;
	float Y;
	float Yaw;
	float V_x;
	float V_y;
	float W;
}PATH_TYPEDEF;

typedef struct ROBOT_CHASSIS
{
	float World_V[3]; // Y , X , W
	float Robot_V[2];
	//float Position[2];
	//float Motor_RPM[4];
	float expect_angle ;
	float Angle;
	int flag;//和底盘通讯用
	
} ROBOT_CHASSIS;

typedef struct
{
	float Distance;
	float Pstart;        //开始位置
	float Pend;          //结束位置
	float Vstart;        //开始的速度           
	float Vmax;          //最大的速度
	float Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
	int flag;            //完成标志位，停下来的时候置1
}TrapezoidPlaning_TYPEDEF;

void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);
int PDControllertest(float x,float y,float w);
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw);
int YawAdjust(float Target_angle);
void AngleLimit(float *angle);
//void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta);
int moving_point_track(float POS_X, float POS_Y, float POS_YAW,float V_max);
void Move_Init(void);
int near_pillar(float x,float y,float POS_YAW,float V_max);
int TrapezoidPlaning(float real_time,float T,float POS_X,float POS_Y,float POS_YAW,float V_start,float V_end,float V_max,float R_ac, float R_de,int* first_time_flag);

int chassis_TrapezoidPlaning(float POS_X_start,
	                    float POS_Y_start,
											float POS_X_end,
											float POS_Y_end,
											float POS_YAW,
											float V_start,
											float V_end,
											float V_max,
											float R_ac,
											float R_de);
int laser_speed_control(float Xstart,float Ystart,//开始时激光的数值
												float Xend,  float Yend,  //结束时目标的激光数值
												int Xnow,  int Ynow,	//实时的激光数值
												float Vmax,  float Vstart,float Vend,float POS_YAW);//最大速度，开始速度，结束速度			

int chassis_TrapezoidPlaning_laser(float POS_X_start,//开始时激光的数值
	                    float POS_Y_start,//开始时激光的数值
											float POS_X_end,//结束时目标的激光数值
											float POS_Y_end,//结束时目标的激光数值
											float POS_YAW,//角度值
											float V_start,//开始速度
											float V_end,//结束速度
											float V_max,//最大速度
											float R_ac,//
											float R_de,//
												int POS_X_now,//
                        int POS_Y_now)	;//											
											
extern PID point_pid;//点对点追踪PID
extern PID yaw_pid;//角度PID
extern ROBOT_CHASSIS Robot_Chassis;

//extern TrapezoidPlaning_TYPEDEF TPlaning_DATA;
/* define ------------------------------------------------------------------*/
//三全向轮底盘的参数
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)

// Chassis Config
#define WHEEL_R            0.076f	                  //轮子半径(单位：m)
#define Robot_R            0.406f                  	//车轮到中心距离(单位：m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //转速与速度的转换 (单位：m/s) 
#define M3508_MS_To_RM     1.0f/(PI*WHEEL_R)      //速度与转速的转换 (单位：m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //轮子直径152mm，电机减速比1:21，轮子一圈pi*152mm
#define RM_transition_MS (PI * WHEEL_R) / 570.0f //转速与速度的转换
#define MS_transition_RM 1.0f / (PI * WHEEL_R) //速度与转速的转换
																										// 计算公式：1/（pi*轮子直径）*减速比*60

//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (32768)
//电机最大转速
#define MAX_MOTOR_SPEED (COUNTS_PER_ROUND*100)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (70.0f)
//定位系统X轴方向到中心距离
#define DISX_OPS2CENTER (0.0f)
//定位系统Y轴方向到中心距离
#define DISY_OPS2CENTER (-307.0f)
//驱动轮减速比
#define WHEEL_REDUCTION_RATIO (2.0f/1.0f)
//M2006减速比
#define M2006_REDUCTION_RATIO (36.0f/1.0f)
//转向齿轮减速比
#define TURNING_REDUCTION_RATIO (4.0f/1.0f)
//轮子转向减速比
#define WHEEL_TURNING_REDUCTION_RATIO (M2006_REDUCTION_RATIO*TURNING_REDUCTION_RATIO)
//底盘旋转半径
#define MOVEBASE_RADIUS (362.039f)
//角度制转化为弧度制
#define ANGLE2RAD(x) (x/180.0f*PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x/PI*180.0f)
																				
//左前轮ID号
#define LEFT_FRONT_ID (1)
//右前轮ID号
#define RIGHT_FRONT_ID (2)
//左后轮ID号
#define LEFT_REAR_ID (3)
//右后轮ID号
#define RIGHT_REAR_ID (4)
//左前轮转向ID号
#define LEFT_FRONT_TURNING_ID (5)
//右前轮转向ID号
#define RIGHT_FRONT_TURNING_ID (6)
//左后轮转向ID号
#define LEFT_REAR_TURNING_ID (7)
//右后轮转向ID号
#define RIGHT_REAR_TURNING_ID (8)

#endif
