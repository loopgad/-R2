#include "control.h"
#include "can1.h"
#include "delay.h"
#include "pstwo.h"
#include "usart.h"
#include "math.h"





int Vx=0,Vy=0,Vz=0;
float Px,Py,Pz;
float PVx,PVy,PVz;
int Motor_A,Motor_B,Motor_C;      
u8 key,mode;


long int wheel_speed[4];
long int encoder[4];
long int last_encoder[4];

long int start_encoder[2];
long int target_encoder[2];
float start_angle;
float target_angle;
int running = 0;
int step = 0;
float error;


int acc_flag = 0;
int break_flag = 0;
int sigmoid_count;
float sigmoid[21]={0.018,0.029 ,0.047 , 0.076 , 0.119 , 0.182 , 0.269 , 
	                 0.377  , 0.500 , 0.622 ,0.731 , 0.817 , 0.881, 0.924 ,
                 	0.952 , 0.970 ,0.982 , 0.989 , 0.993 , 0.996 , 0.997};

/*********机器人坐标系逆运动学************/
void Kinematic_Analysis(float Vx, float Vy, float Vz)
{
	Motor_A   = Vx + L_PARAMETER*Vz;//+yaw*Gyro_K
	Motor_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	Motor_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}

/*********世界坐标系逆运动学*************/
void World_Kinematic_Analysis(float Vx, float Vy, float Vz, float theta)  //theta 机器人坐标系x轴与世界坐标系x轴夹角
{
	theta = PI*theta/180.0f;
	Motor_A   = cos(theta)*Vx         + sin(theta)*Vy        + L_PARAMETER*Vz;
	Motor_B   = -cos(PI/3.0f-theta)*Vx + sin(PI/3.0f-theta)*Vy + L_PARAMETER*Vz;
	Motor_C   = -cos(PI/3.0f+theta)*Vx - sin(PI/3.0f+theta)*Vy + L_PARAMETER*Vz;
}

/**************世界坐标系正运动学*****************/
void World_Forward_Kinematic_Analysis(float theta)  //theta 机器人坐标系x轴与世界坐标系x轴夹角
{
	PVx = wheel_speed[0]*2*cos(theta)/3.0f - wheel_speed[1]*(cos(theta)+sqrt(3.0f)*sin(theta))/3.0f - wheel_speed[2]*(cos(theta)-sqrt(30)*sin(theta))/3.0f ;
	PVy = wheel_speed[0]*2*sin(theta)/3.0f - wheel_speed[1]*(sin(theta)-sqrt(3.0f)*cos(theta))/3.0f - wheel_speed[2]*(sin(theta)+sqrt(30)*cos(theta))/3.0f ;
	PVz = wheel_speed[0]/(3.0f*Radius)     + wheel_speed[1]/(3.0f*Radius)                           + wheel_speed[2]/(3.0f*Radius);
}

void Get_encoder(void)
{
	int i;
	for(i=0;i<4;i++)
	{
		last_encoder[i] = encoder[i];
	  encoder[i] = Real_Position_Value[i];
//		wheel_speed[i] = (encoder[i] - last_encoder[i]) / ( pulse_per_centimeter * dt );  // cm/s
	}
}


void PStwo_control(void)
{ 
	if(PS2_RedLight()==0)
	{
	key=PS2_DataKey();
	Vx=PS2_AnologData(PSS_LX)-127;
	Vy=128-PS2_AnologData(PSS_LY);
	Vz=127-PS2_AnologData(PSS_RX);

		} 
	Vx *= amplify;
	Vy *= amplify;
	Vz *= amplify;
}

void throw_ball(long int start_pos, long int acc_end_pos, long int break_pos, long int stop_pos, long int throw_speed)
{
	if(encoder[3]<acc_end_pos){
		CAN_RoboModule_DRV_Velocity_Position_Mode(0, 4, 5000, 500, acc_end_pos);
	}
	if(encoder[3]>=acc_end_pos&&encoder[3]<break_pos){
		CAN_RoboModule_DRV_Velocity_Position_Mode(0, 4, 5000, throw_speed, break_pos);
	}
	if(encoder[3]>=break_pos && encoder[3]<stop_pos){
		CAN_RoboModule_DRV_Velocity_Position_Mode(0, 4, 5000, 500, stop_pos);
	}
}

void return_start_pos(long int start_pos)
{
	CAN_RoboModule_DRV_Velocity_Position_Mode(0, 4, 500, 500, start_pos);//pwm500,speed 500
}


