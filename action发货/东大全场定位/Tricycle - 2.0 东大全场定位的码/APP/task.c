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

/*****************�궨�������������*********************/

//#define robot_coordinate      //����������ϵ
//#define remote                  //����ң��
#define tracking
#define base_movement        //��������˶�


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
	uart_init(115200);			//��ʼ�����ڲ�����Ϊ115200 
	pid_uart_init(115200);
	Action_uart_init(115200);

	
	Timer2_Init(1000);//5ms..Timer2_Init(1000);
	Nvic_Init();
	
	CAN1_Configuration();                               //CAN1��ʼ��   
	delay_ms(100);
	CAN_RoboModule_DRV_Reset(0,0);                      //��0��1�����������и�λ
	delay_ms(500);                                     //���͸�λָ������ʱ����Ҫ�У��ȴ���������λ��ϡ�
	CAN_RoboModule_DRV_Config(0,0,5,0);               //����Ϊ1s����һ������
	delay_ms(50); 
	CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //ѡ������ٶ�ģʽ
	delay_ms(500);	
	
	#ifdef remote
	PS2_Init();			 //�����˿ڳ�ʼ��
	PS2_SetInit();		 //�����ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
  #endif
	
	usmart_dev.init(84); 	//��ʼ��USMART

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

	/**********Ͷ��**********/
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
	//10ms�Ե������һ�ο���ָ��
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

