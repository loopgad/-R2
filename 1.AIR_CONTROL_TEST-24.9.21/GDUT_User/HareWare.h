#ifndef __HAREWARE_H
#define __HAREWARE_H

#include "elmo.h"
#include "stm32f4xx.h"

/* define --------------------------------------------------------------------*/


//Functions
extern int location_x;
extern int location_y;
extern int location_k;
extern int location_b;
extern uint32_t TIME_ISR_CNT;
extern uint32_t LAST_TIME_ISR_CNT;
extern uint16_t Microsecond_Cnt;

void Air_Pump_Control(uint16_t buf[10]);
void Update_Action(float value[6]);
void Action_Reset(void);
//struct
typedef struct
{
	struct  //ң��ԭʼ���ݣ�8ͨ��
	{
	 uint16_t roll;			//��ҡ��
	 uint16_t pitch;		//
	 uint16_t thr;
	 uint16_t yaw;
	 uint16_t AUX1;
	 uint16_t AUX2;
	 uint16_t AUX3;
	 uint16_t AUX4; 
	 uint16_t	BUX1;
	 uint16_t	BUX2;		
	}Remote; 

}Air_Contorl;


typedef struct ACTION_GL_POS//action����
{

	float POS_X;
	float POS_Y;
	float YAW;
	float W_Z;

	float LAST_POS_X;
	float LAST_POS_Y;
	float LAST_YAW;

	float DELTA_POS_X;
	float DELTA_POS_Y;
	float DELTA_YAW;	
	
	float REAL_X;
	float REAL_Y;
	float REAL_YAW;
	
	float OFFSET_YAW;
	
	
} ACTION_GL_POS;

// �����˵���ʵλ��
typedef struct ROBOT_REAL_POS
{
  float POS_X;
  float POS_Y;     
  float POS_YAW;
  int robot_location;
	
}ROBOT_REAL_POS;

typedef struct ROBOT_CHASSIS
{
	float World_V[3]; // Y , X , W
	float Robot_V[3]; // Y , X , W
	//float Position[2];
	//float Motor_RPM[4];
	float expect_angle ;
	float Angle;
	int flag;//�͵���ͨѶ��
	
} ROBOT_CHASSIS;

extern ROBOT_CHASSIS Robot_Chassis;

extern volatile float value[6];
extern Air_Contorl  Device;
extern ACTION_GL_POS ACTION_GL_POS_DATA;
extern float OFFSET_YAW;
extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;
extern struct ROBOT_REAL_POS ROBOT_TARGET_POS_DATA;

extern uint16_t Time_Sys[4];

//define
#define SWA		PPM_Databuf[4]				//AUX4 1000~2000//û��
#define SWB		PPM_Databuf[5]				//AUX2 1000-1500-2000
#define SWC		PPM_Databuf[6]				//AUX3 1000-1500-2000
#define SWD		PPM_Databuf[7]			    //AUX1 1000~2000

#define ROCK_R_X			PPM_Databuf[3]				//YAW  1000~1500~2000
#define ROCK_R_Y			PPM_Databuf[2]				//THR  1000~1500~2000
#define ROCK_L_Y			PPM_Databuf[1]				//ROLL 1000~1500~2000     
#define	ROCK_L_X		  PPM_Databuf[0]				//PITCH 1000~1500~2000P

#define LEFT_BUTTON		Device.Remote.BUX1
#define RIGHT_BUTTON	Device.Remote.BUX2			


//action����У׼
#define INSTALL_ERROR_Y		0//190.3f
#define INSTALL_ERROR_X		0
#define USART_REC_LEN     200   //�����������ֽ��� 200
#define RXBUFFERSIZE   1 //�����С

//΢����־λ


//extern
extern Air_Contorl  Device;
extern ACTION_GL_POS ACTION_GL_POS_DATA;
extern float OFFSET_YAW;
extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;
extern unsigned char aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
extern uint16_t PPM_Databuf[10];//����ͨ��������

extern int PPM_Connected_Flag;

#endif
