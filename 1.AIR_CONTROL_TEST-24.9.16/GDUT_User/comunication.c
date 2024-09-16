/**
  ******************************************************************************
  * @file    comunication.c
  * @author  王茗康 lly
  * @version V1.2.0
  * @date    2023/4/5
  * @brief   
  ******************************************************************************
  */ 

#include "comunication.h"
#include "HareWare.h" 
#include "moto.h"
#include "MoveBase.h"
#include "calculation.h"
#include "FSM.h"
#include "robot.h"
#include "Communication_STM32.h"


float data1=0;
float data2=0;
float data3=0;
float data4=0;
float data5=0;
float data6=0;
float data7=0;
float data8=0;
float data9=0;
float data10=0;


//数据接收暂存区
//unsigned char  receiveBuff[50] = {0};
//unsigned char  buf[50] = {0};//发送用
unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};
//接收数据
unsigned char USART1_Receiver             = 0;          
//unsigned char USART2_Receiver             = 0;          
//unsigned char USART3_Receiver             = 0;          
unsigned char UART4_Receiver             = 0;          
unsigned char UART5_Receiver             = 0;          
unsigned char USART6_Receiver             = 0;          
union receiveData1
{
	int d;
	unsigned char data[4];
}x_position,y_position,k_position,b_position;


/**
  * @brief  计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
  * @param   数组地址、数组大小
  * @retval 
  */
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}

/**
  * @brief  四个激光测距usart1
  * @param   返回四个激光的信息
  * @retval 
  */
//做了修改，修改用于四个激光测距
unsigned char  receiveBuff_u1[22] = {0};
int VisionReceiveData_1(int *theta_angle,int *theta_pitch,int *theta_yaw,int *theta_ras)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;
  HAL_UART_Receive_IT(&huart1, &USART1_Receiver, 1); // 继续监听
	
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART1_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				//printf("header ok\n");
				receiveBuff_u1[0]=header[0];         //buf[0]
				receiveBuff_u1[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = USART1_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收数据的长度
				receiveBuff_u1[2] = USART1_Receiver;
				dataLength     = receiveBuff_u1[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff_u1[j + 3] = USART1_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://接收校验值信息(设定为0x07)
				receiveBuff_u1[2 + dataLength] = USART1_Receiver;
				checkSum = getCrc8(receiveBuff_u1, 3 + dataLength);
//				checkSum = 0x07;
				  // 检查信息校验值
//				if (checkSum != receiveBuff[3 + dataLength]) //buf[11]
//				{
////					printf("Received data check sum error!");
//					return 0;
//				}
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					//数据0d     buf[11]  无需判断
					k++;
				}
				else if (k==1)
				{
					//数据0a     buf[12] 无需判断

					//进行速度赋值操作					
					 for(k = 0; k < 4; k++)
					{
						x_position.data[k]  = receiveBuff_u1[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						y_position.data[k] = receiveBuff_u1[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						k_position.data[k]  = receiveBuff_u1[k + 11]; //buf[3]  buf[4] buf[5]  buf[6]
						b_position.data[k] = receiveBuff_u1[k + 15]; //buf[7]  buf[8] buf[9]  buf[10]
						
						
					}				
//					if(x_position.d==12)
//					{
//						printf("OK!");
//					}
//					else
//					{
//						printf("error!");
//					}
					//赋值操作
					*theta_angle = x_position.d;//偏航角
					*theta_pitch = y_position.d;//俯仰角
					*theta_yaw = k_position.d;
					*theta_ras = b_position.d;
//					*theta =(int)angle.d;
//					
//					//ctrlFlag
//					*flag = receiveBuff[9];                //buf[9]
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}

/**
  * @brief  使用uart4和底盘通讯（接受）
  * @param   将action数据传入ROBOT_REAL_POS_DATA
  * @retval 
  */
//接受联合体
unsigned char  receiveBuff_u4[22] = {0};
union uart4_ReceiveData
{
	float d;
	unsigned char data[4];
}real_x,real_y,real_w;

union uart4_ReceiveData_int
{
	int d;
	unsigned char data[4];
}chassic_flag;
int uart4_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	HAL_UART_Receive_IT(&huart4, &UART4_Receiver, 1); // 继续监听
	//接收消息头
	if(Start_Flag == START)
	{
		if(UART4_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				receiveBuff_u4[0]=header[0];         //buf[0]
				receiveBuff_u4[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = UART4_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收数据的长度
				receiveBuff_u4[2] = UART4_Receiver;
				dataLength     = receiveBuff_u4[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff_u4[j + 3] = UART4_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://接收校验值信息(设定为0x07)
				receiveBuff_u4[2 + dataLength] = UART4_Receiver;
				checkSum = getCrc8(receiveBuff_u4, 3 + dataLength);
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					k++;
				}
				else if (k==1)
				{				
					 for(k = 0; k < 4; k++)
					{
						real_x.data[k]  = receiveBuff_u4[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						real_y.data [k] = receiveBuff_u4[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						real_w.data [k]  = receiveBuff_u4[k + 11]; //buf[3]  buf[4] buf[5]  buf[6]
						chassic_flag.data[k] = receiveBuff_u4[k + 15]; //buf[7]  buf[8] buf[9]  buf[10]
					}				

					//赋值操作
					*action_x = -real_x.d;
					*action_y = -real_y.d;
					*action_w = real_w.d;
					*flag = chassic_flag.d;
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**
  * @brief  使用uart4和底盘通讯（发送）
* @param   x,y,w皆为世界坐标系下的速度，flag用来发送其他数据
  * @retval 
  */
//unsigned char buf[22]={0};//数据缓存区

union Uart4_SendData//发送数据的共用体
{
	float d;
	unsigned char data[4];
}uart4_vx,uart4_vy,uart4_vw;

union Uart4_SendData_int//发送数据的共用体
{
	int d;
	unsigned char data[4];
}uart4_flag;



void Usart4_SendData(float X,float Y,float W,int flag)
{
	
	int i,length = 0;
	unsigned char  buf_u4[22] = {0};
	 //memset(buf,0,50);//清空数组
	uart4_vx.d = X;
	uart4_vy.d = Y;
	uart4_vw.d = W;
	uart4_flag.d = flag;
	for(i=0;i<2;i++)
	{
		buf_u4[i]=header[i];//协议数据头
	}
	length = 17;
	buf_u4[2] =length;//sizeof
	for(i=0;i<4;i++)
	{
		buf_u4[i+3]=uart4_vx.data[i];
		buf_u4[i+7]=uart4_vy.data[i];
		buf_u4[i+11]=uart4_vw.data[i];
		buf_u4[i+15]=uart4_flag.data[i];
		
		
	}
	buf_u4[3+length-1]=getCrc8(buf_u4,3+length);
	buf_u4[3+length]=ender[0];
	buf_u4[3+length+1]=ender[1];
	
	UART4_Send_String(buf_u4,sizeof(buf_u4));//利用字符串发送函数发送数据
}




/**
  * @brief  使用uart4和右发射机构通讯（接受）
  * @param   将action数据传入ROBOT_REAL_POS_DATA
  * @retval 
  */
//接受联合体
unsigned char  receiveBuff_u5[22] = {0};
union uart5_ReceiveData
{
	int d;
	unsigned char data[4];
}gun1,gun2,gun3,gun_flag;

int uart5_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	HAL_UART_Receive_IT(&huart5, &UART5_Receiver, 1); // 继续监听
	//接收消息头
	if(Start_Flag == START)
	{
		if(UART5_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				receiveBuff_u5[0]=header[0];         //buf[0]
				receiveBuff_u5[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = UART4_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收数据的长度
				receiveBuff_u5[2] = UART5_Receiver;
				dataLength     = receiveBuff_u5[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff_u5[j + 3] = UART5_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://接收校验值信息(设定为0x07)
				receiveBuff_u5[2 + dataLength] = UART5_Receiver;
				checkSum = getCrc8(receiveBuff_u5, 3 + dataLength);
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					k++;
				}
				else if (k==1)
				{				
					 for(k = 0; k < 4; k++)
					{
						x_position.data[k]  = receiveBuff_u5[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						y_position.data[k] = receiveBuff_u5[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						k_position.data[k]  = receiveBuff_u5[k + 11]; //buf[3]  buf[4] buf[5]  buf[6]
						b_position.data[k] = receiveBuff_u5[k + 15]; //buf[7]  buf[8] buf[9]  buf[10]
					}				

					//赋值操作
					*action_x = real_x.d;
					*action_y = real_y.d;
					*action_w = real_w.d;
					*flag = chassic_flag.d;
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**
  * @brief  使用uart5和底盘通讯（发送）
* @param   x,y,w皆为世界坐标系下的速度，flag用来发送其他数据
  * @retval 
  */
//unsigned char buf[22]={0};//数据缓存区

union Uart5_SendData//发送数据的共用体
{
	float d;
	unsigned char data[4];
}uart5_vx,uart5_vy,uart5_vw,uart5_flag;
void Usart5_SendData(float X,float Y,float W,int flag)
{
	unsigned char  buf_u5[22] = {0};
	int i,length = 0;
	uart4_vx.d = X;
	uart4_vy.d = Y;
	uart4_vw.d = W;
	uart4_flag.d = flag;
	for(i=0;i<2;i++)
	{
		buf_u5[i]=header[i];//协议数据头
	}
	length = 17;
	buf_u5[2] =length;//sizeof
	for(i=0;i<4;i++)
	{
		buf_u5[i+3]=uart5_vx.data[i];
		buf_u5[i+7]=uart5_vy.data[i];
		buf_u5[i+11]=uart5_vw.data[i];
		buf_u5[i+15]=uart5_flag.data[i];
		
		
	}
	buf_u5[3+length-1]=getCrc8(buf_u5,3+length);
	buf_u5[3+length]=ender[0];
	buf_u5[3+length+1]=ender[1];
	
	UART5_Send_String(buf_u5,sizeof(buf_u5));//利用字符串发送函数发送数据
}


/*
字符串发送函数1-6
发送指定大小的字符数组
入口参数：数组地址、数组大小
*/

void USART1_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//静态变量防止数据丢失
	while(length<sendSize)
	{
		while(!(USART1->SR&(0x01<<7)));//发送缓冲区为空(发送数据缓冲位应该为第八位)
		USART1->DR=*p;
		p++;
		length++;
	}
	length=0;
}

void UART4_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//静态变量防止数据丢失
	while(length<sendSize)
	{
		while(!(UART4->SR&(0x01<<7)));//发送缓冲区为空(发送数据缓冲位应该为第八位)
		UART4->DR=*p;
		p++;
		length++;
	}
	length=0;
}

void UART5_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//静态变量防止数据丢失
	while(length<sendSize)
	{
		while(!(UART5->SR&(0x01<<7)));//发送缓冲区为空(发送数据缓冲位应该为第八位)
		UART5->DR=*p;
		p++;
		length++;
	}
	length=0;
}


/**
  * @brief  串口中断回调函数
	* @param  
	* @retval None
  * @attention
  */
//机器人姿态接受数据联合体
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;
//	static uint8_t count = 0;
	//static uint8_t i = 0;
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
//usart1
//激光
	if(huart->Instance==USART1)
	{
	 VisionReceiveData_1(&location_y,&location_x,&location_k,&location_b);
		

	 	
	 HAL_UART_Receive_IT(&huart1, &USART1_Receiver , 1); // 继续监
	}
//usart4
//底盘
	if(huart->Instance==UART4)//如果是串口4.
	{
	//将action数据传入ROBOT_REAL_POS_DATA
	
	Update_Action();
	HAL_UART_Receive_IT(&huart4, &UART4_Receiver, 1);
	}
		if(huart->Instance==UART5)//如果是串口5
	{
	//将action数据传入ROBOT_REAL_POS_DATA
	//uart4_ReceiveData(&ROBOT_REAL_POS_DATA.POS_X,&ROBOT_REAL_POS_DATA.POS_Y,&ROBOT_REAL_POS_DATA.POS_YAW,&ROBOT_FLAG.Chassis_receive);
	HAL_UART_Receive_IT(&huart5, &UART5_Receiver, 1);
	}
 
	if(huart->Instance==USART2)
	{
		STM32_READ_FROM_ROS(&data1, &data2, &data3, &data4, &data5, &data6, &data7, &data8, &data9, &data10);

	HAL_UART_Receive_IT(&huart2, &USART_Receiver2 , 1); // 继续监
	}
}



/*----------------------------------------激光------------------------------------------------------*/
//激光平均值滤波
averageFilter_TPYE laser_filter_y;
averageFilter_TPYE laser_filter_x;
averageFilter_TPYE laser_filter_k;
averageFilter_TPYE laser_filter_b;
float real_location_y;
float real_location_x;
float real_location_k;
float real_location_b;
//修改用于发射定位
int Laser_calibration(float x, float y,float yaw,float v_max)
{
	//激光滤波
	laser_filter_x.indata=location_x;
	laser_filter_y.indata=location_y;
	averageFilter(&laser_filter_x);
	averageFilter(&laser_filter_y);
	real_location_x=laser_filter_x.outdata;
	real_location_y=laser_filter_y.outdata;

	float Laser_error_x,Laser_error_y,ERROR_SHOOTING;
	
	YawAdjust(yaw);
	//转到指定角度
	Laser_error_x =  location_x - x;
	Laser_error_y =  location_y - y;
//	Laser_error_x =  real_location_x - x;
//	Laser_error_y =  real_location_y - y;
	
	ERROR_SHOOTING=sqrt(Laser_error_x*Laser_error_x+Laser_error_y*Laser_error_y);
//判断距离是否合适
	if(Laser_error_x<500&&Laser_error_y<10000)
	{
		if(ABS(Laser_error_x)>5.0f||ABS(Laser_error_y)>5.0f||ABS(yaw-ROBOT_REAL_POS_DATA.POS_YAW)>0.8)
		{
//pid直接输出 
			laser_K_pid.outputmax = ABS(v_max);
			PID_position_PID_calculation_by_error(&laser_B_pid, ERROR_SHOOTING);

			Robot_Chassis.World_V[1] = MaxMinLimit(-laser_B_pid.output*0.1f*Laser_error_x/sqrt(ERROR_SHOOTING),v_max);
			Robot_Chassis.World_V[0]= MaxMinLimit(laser_B_pid.output*0.1f*Laser_error_y/sqrt(ERROR_SHOOTING),v_max);
			
		Laser_error_x =  location_x - x;
	  Laser_error_y =  location_y - y;
//			Laser_error_x =  real_location_x - x;
//	    Laser_error_y =  real_location_y - y;
		}
		else
		{
			//ACTION_GL_POS_DATA.REAL_X=218.0f;
			//ACTION_GL_POS_DATA.REAL_Y=905.0f;
			return 1;
		}
	}
   
	return 0;
	
}
//修改用于取环定位
int Laser_calibration_1(float k, float b,float yaw,float v_max,int direction_1)
{
		//激光滤波
	laser_filter_k.indata=location_k;
	laser_filter_b.indata=location_b;
	averageFilter(&laser_filter_k);
	averageFilter(&laser_filter_b);
	real_location_k=laser_filter_k.outdata;
	real_location_b=laser_filter_b.outdata;
	float error_K,error_B,ERROR;

	YawAdjust(yaw);
	//转到指定角度
	error_K =  location_k - k;
	error_B =  location_b - b;
//	error_K =  real_location_k - k;
//	error_B =  real_location_b - b;
	ERROR=sqrt(error_K*error_K+error_B*error_B);
//判断距离是否合适
	if(error_K<1500&&error_B<1500)
	{
		if(ABS(error_K)>8.0f||ABS(error_B)>8.0f||ABS(yaw-ROBOT_REAL_POS_DATA.POS_YAW)>0.8f)
		{
//pid直接输出 
//增加限幅
			laser_K_pid.outputmax = ABS(v_max);
			PID_position_PID_calculation_by_error(&laser_K_pid, ERROR);
			if(direction_1==1)
			{
				
			Robot_Chassis.World_V[0] = MaxMinLimit(-laser_K_pid.output*1.0f*error_K/ERROR,v_max);
			Robot_Chassis.World_V[1] = MaxMinLimit(-laser_K_pid.output*1.0f*error_B/ERROR,v_max);
			}
			if(direction_1==2)
			{
			Robot_Chassis.World_V[1] = MaxMinLimit(laser_K_pid.output*1.0f*error_K/ERROR,v_max);
			Robot_Chassis.World_V[0] = MaxMinLimit(-laser_K_pid.output*1.0f*error_B/ERROR,v_max);
			}
			error_K =  location_k - k;
			error_B =  location_b - k;
//			 error_K =  real_location_k - k;
//	     error_B =  real_location_b - b;
			
		}
		//else if(ABS(yaw-ROBOT_REAL_POS_DATA.POS_YAW)>1){}
		else return 1;
			
	}
   
	return 0;
}
	



/******************************************************************************************************/

/**************************************************/
///*
////发送数据
//*/
//unsigned char buf[14]={0};//数据缓存区

//union sendData//发送数据的共用体
//{
//	int d;
//	unsigned char data[4];
//}dr_x,dr_y,dr_yaw;
//void VisionSendData(int X,int Y)
//{
//	
//	int i,length = 0;
//	
//	//左右轮期望速度，未知
//	dr_x.d = X;
//	dr_y.d = Y;

//	
//	for(i=0;i<2;i++)
//	{
//		buf[i]=header[i];//协议数据头
//	}
//	length = 9;
//	buf[2] =length;//sizeof
//	for(i=0;i<4;i++)
//	{
//		buf[i+3]=dr_x.data[i];
//		buf[i+7]=dr_y.data[i];
//		
//	}
//	buf[3+length-1]=getCrc8(buf,3+length);
//	buf[3+length]=ender[0];
//	buf[3+length+1]=ender[1];
//	
//	USART_Send_String(buf,sizeof(buf));//利用字符串发送函数发送数据
//}
///*
//字符串发送函数
//发送指定大小的字符数组
//入口参数：数组地址、数组大小
//*/
//void USART_Send_String(uint8_t *p,uint16_t sendSize)
//{
//	static int length=0;//静态变量防止数据丢失
//	while(length<sendSize)
//	{
//		while(!(USART1->SR&(0x01<<7)));//发送缓冲区为空(发送数据缓冲位应该为第八位)
//		USART1->DR=*p;
//		p++;
//		length++;
//	}
//	length=0;
//}
