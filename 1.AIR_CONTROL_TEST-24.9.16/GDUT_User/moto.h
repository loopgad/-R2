/**
  ******************************************************************************
  * @file    
  * @author  梁立言
  * @version 1.1.0
  * @date    2023/3/29
  * @brief   电机控制库
  ******************************************************************************
  * @attention
  *速度控制和位置控制可以用，回零模式和转矩模式也可以用，但是注意转矩模式下电机可能会出现抖动
	*曲线规划和梯形规划还在移植中，别急，如果急可以帮我移植先
	*我把刘家隆的运动解算删掉了，他很生气，谁有空帮我把他移回来
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTO_H
#define __MOTO_H
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
//#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "PID.h"
#include "moto.h"
#include "math.h"
#include "can.h"
/* define --------------------------------------------------------------------*/
// M3508电机编号
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203
#define M3508_CHASSIS_MOTOR_ID_4      0x204
#define M3508_CHASSIS_MOTOR_ID_5      0x205
#define M3508_CHASSIS_MOTOR_ID_6      0x206
#define M2006_CHASSIS_MOTOR_ID_0			0x207
#define M2006_CHASSIS_MOTOR_ID_1			0x208

#define L 0.1730f
#define number 1.7320f		//sqrt(3)
#define PI 3.14159265358979323846


//驱动器工作模式
#define  SPEED_CONTROL_MODE				2
#define  VELOCITY_PLANNING_MODE   3
#define  CURRENT_MODE             4
#define  POSITION_CONTROL_MODE		5
#define  SPEED_TARQUE_CONTROL_MODE 6
#define  POSITION_TORQUE_MODE		  7
#define  HOMEINGMODE		          9
#define  MOTO_OFF		              0



/*******************************控制驱动器命令************************************/

/** 
  * @brief  电机种类  3508 or 2006
  */   
typedef enum
{ 
  RM_3508   = 1, 
  M_2006    = 2,
	NONE      = 3  //none表示没有接电机

}MotorType_TypeDef;

/** 
  * @brief  currentType structure definition  
  * @note     
  */
typedef struct
{
	float current;
	
	int32_t cnt;
	
	int flag;
}currentType;

/** 
  * @brief  HomingMode type structure definition  
  * @note     
  */
typedef struct
{
	float vel;
	
	float current;
	
	float initPos;//重置初始位置，暂时无用
	
	int32_t cnt;
	
	int flag;
}HomingModeType;

/** 
  * @brief  Pos_TorqueMode type structure definition  
  * @note     
  */
typedef struct
{
	float current;
	
	float Pos;//目标位置
	
	float output;
	
	int16_t  TARGET_TORQUE;//目标转矩，用电流表示
	
	int flag;
	
	int32_t cnt;
}Pos_TorqueModeType;

/** 
  * @brief  Speed_TorqueMode type structure definition  
  * @note     
  */
typedef struct
{
	float current;
	
	float Vel;//目标速度
	
	float output;
	
	int16_t  TARGET_TORQUE;//目标转矩，用电流表示
	
	int flag;
	
	int32_t cnt;
}Vel_TorqueModeType;

/** 
  * @brief   VELOCITY_PLANNING type structure definition  
  * @note     
  */
typedef struct VELOCITY_PLANNING //速度规划
{
	float Distance;
	float Pstart;        //开始位置
	float Pend;          //结束位置
	float Vstart;        //开始的速度           // 单位：RPM 绝对值
	float Vmax;          //最大的速度
	float Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
	int flag;            //完成标志位，电机停下来的时候置1
}VELOCITY_PLANNING;

/** 
  * @brief  电机信息  
  * @note     
  */
typedef struct MOTO_REAL_INFO
{
	// 电机模式
	uint32_t unitMode;//电机模式
		//POSITION_CONTROL_MODE位置模式
		//POSITION_TARQUE_CONTROL_MODE位置_力度模式
	  //SPEED_TARQUE_CONTROL_MODE位置_力度模式
		//SPEED_CONTROL_MODE速度模式
		//MOTO_OFF电机关闭-->电流不发送
	  //VELOCITY_PLANNING_MODE梯形规划模式
	//
	MotorType_TypeDef type;//电机类型：m3508、m2006
	uint16_t ANGLE;   		//采样角度						
	int16_t  RPM;					//速度值			
	int16_t  CURRENT;     //电流值
	int16_t  TARGET_CURRENT;//目标电流值
	int16_t  TARGET_POS;//目标角度
	float    TARGET_RPM;//目标转速
	int      Velflag;//数度为零时，置1 
	//结构体
	HomingModeType homingMode;//电机回零模式
	Pos_TorqueModeType pos_torquemode;//位置转矩模式
	Vel_TorqueModeType vel_torquemode;//速度转矩模式
	VELOCITY_PLANNING velocity_planning;//速度规划

	
	// 角度积分时用到下面变量
	float		 REAL_ANGLE;         //处理过的真实角度（必须用float）
	uint8_t	 FIRST_ANGLE_INTEGRAL_FLAG;  //?
	uint16_t LAST_ANGLE;   //?
	int16_t filter_RPM;
}MOTO_REAL_INFO;

extern MOTO_REAL_INFO MOTOR_REAL_INFO[8]; 

//电机的目标速度
typedef struct MOTOR_RPM
{
	float MOTOR1_RPM;
	float MOTOR2_RPM;
	float MOTOR3_RPM;
	float MOTOR4_RPM;
	float MOTOR5_RPM;
	float MOTOR6_RPM;
	float MOTOR7_RPM;
	float MOTOR8_RPM;
}MOTOR_RPM;

//电机的目标位置
typedef struct MOTOR_POS
{
	float MOTOR1_POS;
	float MOTOR2_POS;
	float MOTOR3_POS;
	float MOTOR4_POS;
	float MOTOR5_POS;
	float MOTOR6_POS;
	float MOTOR7_POS;
	float MOTOR8_POS;
}MOTOR_POS;


//用于曲线规划的结构体
//用不了
/* 定义电机速度曲线对象 */
typedef struct CurveObject {
  float startSpeed;    //开始调速时的初始速度
  float currentSpeed;   //当前速度
  float targetSpeed;    //目标速度
  float stepSpeed;     //加速度
  float speedMax;     //最大速度
  float speedMin;     //最小速度
  uint32_t aTimes;     //调速时间
  uint32_t maxTimes;    //调速跨度
	float  p_add;    //加速的占比
	float  p_decrease; //减速的占比
  
}CurveObjectType;




/* Exported types ------------------------------------------------------------*/
extern struct PID MOTOR_PID_POS[8];				// 位置环
extern struct PID MOTOR_PID_RPM[8];	// 1-4底盘电机 5-6发射电机
extern struct PID laser_X_pid;//激光pid
extern struct PID laser_Y_pid;
extern struct PID laser_K_pid;
extern struct PID laser_B_pid;
extern struct MOTO_REAL_INFO MOTOR_REAL_INFO[8]; 
extern struct MOTOR_RPM MOTOR_TARGET_RPM;
extern struct MOTOR_POS MOTOR_TARGET_POS;


//extern pid yaw_pid;//角度PID
//extern float YawAdjust_error;


/* Exported functions ------------------------------------------------------- */
float MaxMinLimit(float val,float limit);
void M3508_Motor_Init(void);
void m3508_update_m3508_info(CAN_RxHeaderTypeDef *msg,uint8_t	can1_RxData[8]);
void chassis_m3508_send_motor_currents(void);//发送电流
void M3508AngleIntegral(MOTO_REAL_INFO *M3508_MOTOR);//角度积分
void MotorCtrl(void);//数据计算
float CurrentCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_current);
float VelCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_vel);
float PosCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_pos);
void HomingMode(MOTO_REAL_INFO *MOTOR_REAL_INFO);
void Pos_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_pos);
void Vel_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_Vel);
float MaxMinLimit(float val,float limit);
void VelocityPlanningMODE(MOTO_REAL_INFO *M3508_MOTOR);
float position_control_an_delay(PID *pp_po,PID *pp_rpm,MOTO_REAL_INFO *info,float v_xian,float tar_position,float current_key,int dalay_time,int next_flag,int first_flag);
void Velocity_Planning_setpos(MOTO_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde);
double filter(double input, double prev_output, double prev_input, double cutoff_freq, double sample_rate) ;
void calculateWheelSpeeds(double world_vx, double world_vy, double omega, float *RPM1, float *RPM2, float *RPM3);


int YawAdjust(float Target_angle);
void AngleLimit(float *angle);

#endif
/***********************END OF FILE*************/

