#ifndef __COMUNICATION_H
#define __COMUNICATION_H
//#include "usart.h"
#define START 0X11

extern float data1;
extern float data2;
extern float data3;
extern float data4;
extern float data5;
extern float data6;
extern float data7;
extern float data8;
extern float data9;
extern float data10;
extern unsigned char USART1_Receiver          ;          //接收数据
//extern unsigned char USART2_Receiver          ; 
//extern unsigned char USART3_Receiver          ; 
extern unsigned char UART4_Receiver          ; 
extern unsigned char UART5_Receiver          ; 
extern unsigned char USART6_Receiver          ; 

unsigned char getCrc8(unsigned char *ptr,unsigned short len);
int VisionReceiveData_1(int *theta_angle,int *theta_pitch,int *theta_yaw,int *theta_ras);
//int Laser_calibration(float x, float y,float yaw,float v_max);
//int Laser_calibration_1(float k, float b,float yaw,float v_max,int direction);
int uart4_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag);
int uart5_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag);
//void USART_Send_String(uint8_t *p,uint16_t sendSize);
void VisionSendData(int X,int Y);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void Usart4_SendData(float X,float Y,float W,int flag);
void Usart5_SendData(float X,float Y,float W,int flag);
void USART1_Send_String(uint8_t *p,uint16_t sendSize);
void UART4_Send_String(uint8_t *p,uint16_t sendSize);
void UART5_Send_String(uint8_t *p,uint16_t sendSize);
#endif 
