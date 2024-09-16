#ifndef __STM32_COMMUNICATION_H
#define __STM32_COMMUNICATION_H
#include "main.h"
//#include <usart.h>

enum FLAG{CHECK_HEADER_FLAG, RECEIVE_DATA_FLAG, CHECK_VALUE_FLAG, FINAL_RECEIVE_FLAG};
int STM32_READ_FROM_ROS(float *data1, float *data2, float *data3, float *data4, float *data5, float *data6, float *data7, float *data8, float *data9, float *data10);
void STM32_WRITE_TO_ROS(int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8, int data9, int data10);
unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len);
extern unsigned char USART_Receiver2;
#endif 
