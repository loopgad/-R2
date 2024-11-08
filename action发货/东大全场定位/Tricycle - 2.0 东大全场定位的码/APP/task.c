#include "task.h"
#include "timer2.h"
#include "delay.h"
#include "usart.h"
#include "can1.h"
#include "control.h"
#include "protocol.h"
#include "dma.h"
#include "pstwo.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "Action_usart.h"
//#include "PID.h"
#include "usmart.h"
#include "pid_usart.h"
#include "point.h"

/*****************宏定义决定开启功能*********************/

//#define robot_coordinate      //机器人坐标系
//#define remote                  //开启遥控
#define tracking
#define base_movement        //允许底盘运动


uint8_t Bsp_Int_Ok = 0;
extern int Vx,Vy,Vz;
extern int Motor_A,Motor_B,Motor_C;    


extern float pos_x;
extern float pos_y;
extern float zangle;
extern float w_z;
extern float p1,i1,d1,p2,i2,d2,t1,t2;

void pid_yaw_set(int p,int i,int d, int max)
{
//	pid_yaw.Proportion = p;
//	//pid_yaw.Integral = 0.1;
//	pid_yaw.Derivative = d;
//	pid_yaw.outputmax = max;
}
void pid_position_set(int p,int i,int d, int max)
{
//	pid_position.Proportion = p;
////	pid_position.Integral = i;
//	pid_position.Derivative = d;
//	pid_position.outputmax = max;
}

void goal_set(int angle,int x,int y)
{
//	goal_angle  = angle;
//	goal_pos[0] = x;
//	goal_pos[1] = y;
}


void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
}

void BSP_Init(void)
{
	uart_init(115200);			//初始化串口波特率为115200 
	pid_uart_init(115200);
	Action_uart_init(115200);

	
	Timer2_Init(1000);//5ms..Timer2_Init(1000);
	Nvic_Init();
	
	CAN1_Configuration();                               //CAN1初始化   
	delay_ms(100);
	CAN_RoboModule_DRV_Reset(0,0);                      //对0组1号驱动器进行复位
	delay_ms(500);                                     //发送复位指令后的延时必须要有，等待驱动器复位完毕。
	CAN_RoboModule_DRV_Config(0,0,5,0);               //配置为1s传回一次数据
	delay_ms(50); 
	CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //选择进入速度模式
	delay_ms(500);	
	
	#ifdef remote
	PS2_Init();			 //驱动端口初始化
	PS2_SetInit();		 //配配置初始化,配置“红绿灯模式”，并选择是否可以修改
  #endif
	
	usmart_dev.init(84); 	//初始化USMART

	Bsp_Int_Ok = 1;
}

void Task_1000HZ(void)
{
	//control_motor();
}

void Task_500HZ(void)
{
}

void Task_250HZ(void)
{
}

void Task_200HZ(void)
{

	/**********投射**********/
//	Get_encoder();
//	start_pos=7000, acc_end_pos=8000, break_pos=13500, stop_pos=16000, throw_speed=4000;
//	if(key == 8){
//		throw_ball(start_pos, acc_end_pos, break_pos, stop_pos, throw_speed);
//	}
//	if(key == 9){
//		return_start_pos(start_pos);
//	}
	
//	World_Forward_Kinematic_Analysis(Pz);
//	odometry();
//navigation();		
}
void Task_100HZ(void)
{
	//10ms对电机发送一次控制指令
	#ifdef remote
	PStwo_control();
	#endif	

	#ifdef tracking
	use_tracking();
	#endif
	
	
	
  printf(" %f,   %f,    %f   %d,   %d,    %d ,     %f \r\n",pos_x,pos_y,zangle , Vx, Vy, Vz ,t1);
//  printf(" %f,    %f,    %f,    %f,    %f,    %f,    %f,    %f  \r\n",p1,i1,d1,p2,i2,d2,t1,t2 );
	
	#ifdef robot_coordinate
	Kinematic_Analysis(Vx, Vy, Vz);
	#else 
	World_Kinematic_Analysis(Vx, Vy, Vz, zangle);
	#endif
	
	#ifdef base_movement
  CAN_RoboModule_DRV_Velocity_Mode(0,1,5000,Motor_A);
  CAN_RoboModule_DRV_Velocity_Mode(0,2,5000,Motor_B);
  CAN_RoboModule_DRV_Velocity_Mode(0,3,5000,Motor_C);
	#endif

}

void Task_50HZ(void)
{
  send_data();
}

void Task_20HZ(void)
{
	
}
 
void Task_10HZ(void)
{

	
}

void Task_1HZ(void)
{

}

