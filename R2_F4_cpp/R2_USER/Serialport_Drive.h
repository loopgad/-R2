<<<<<<< HEAD
=======
/*
Copyright (c) 2024 loopgad 9th_R2_Member base_author

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
#pragma once

#include "usart.h"
#include <cstdint>
#include <string.h>
<<<<<<< HEAD

=======
#include <stdio.h>
#include "crc_util.h"
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
#ifdef __cplusplus
extern "C" {
#endif

#include "Global_Namespace.h"
#include "crc_util.h"

<<<<<<< HEAD
=======

>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
// Xbox 手柄接收帧定义
#define FRAME_HEAD_0_RC9 0xFC
#define FRAME_HEAD_1_RC9 0xFB
#define FRAME_END_0_RC9 0xFD
#define FRAME_END_1_RC9 0xFE
#define MAX_DATA_LENGTH_RC9 64


class Serialport_Drive 
{
/********************************ROS部分***************************/
 private:
<<<<<<< HEAD
 union ROS_data
	{
		float f;
		uint8_t c[4];
	}x,y,vx,vy;
	
    uint8_t header[2];
    uint8_t tail[2];
    uint8_t lenth=0;		
 public: 
    int8_t Recieve_From_ROS(uint8_t *buffer);
=======
    //数据包头包尾
    char header[2] = {0x55, 0xAA};
    char ender[2] = {0x0D, 0x0A};
    //发送共用体    
	union Send {
        float data;
        unsigned char array[4];
    } ToROSworld_px, ToROSworld_py, ToROSworld_vx, ToROSworld_vy;
    //接收共用体
    union Recieve {
        float data;
        unsigned char array[4];
    } FromROS_NEXTPOINTX, FromROS_NEXTPOINTY;

 public:
    float nextpoint[2]; // 用于接收下一个目标点
    void Send_to_ROS(float wx, float wy, float vx, float vy, UART_HandleTypeDef *huart);
    void GET_ROS_DATA(uint8_t buffer[]);
    unsigned char ros_serial_get_crc8_value(unsigned char *tem_array, unsigned char len);
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
/**************************************************************/

/*****************************Xbox部分************************/
 private:
 // 数据帧结构体
 typedef struct {
    uint8_t data_length; 
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint8_t rx_temp_data_mat[MAX_DATA_LENGTH_RC9]; 
    uint8_t frame_end[2];
    union {
        uint16_t crc_code;
        uint8_t crc_buff[2]; 
    } check_code;
    uint16_t crc_calculated;
 } serial_frame_mat_t;

// 状态机状态（枚举量）
typedef enum {
    WAITING_FOR_HEADER_0,
    WAITING_FOR_HEADER_1,
    WAITING_FOR_ID,
    WAITING_FOR_LENGTH,
    WAITING_FOR_DATA,
    WAITING_FOR_CRC_0,
    WAITING_FOR_CRC_1,
    WAITING_FOR_END_0,
    WAITING_FOR_END_1
} rxState;
 // 初始化变量
 serial_frame_mat_t rx_frame_mat = {}; // 初始化接收数据帧结构体
 uint8_t rxIndex_ = 0; // 当前接收的字节索引
 rxState state_ = WAITING_FOR_HEADER_0; // 从初始状态开始接收

 public:
    void Xbox_Receive_Data(uint8_t byte);
/****************************************************************/

/***************************Action部分************************/
 private:
 union 
 {
    uint8_t data[24];
    float ActVal[6];
 } posture;
		
 uint8_t ch;
 uint8_t count = 0;
 uint8_t i = 0;
 //Action数据缓冲区
 volatile float action_Data[6];

 public:
    void Action_Receive_Data(uint8_t RxBuffer_for3[]);
    void Update_Action(float value[6]);
/*************************************************************/   

/************************串口调试用*****************************/
private:
//使用VOFA的JustFloat数据帧
typedef struct JustFloat{
    float fdata[2] = {0}; //通道数
    unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f}; //包尾
}JustFloat;

JustFloat Debug_Data = {}; //调试使用

public:
<<<<<<< HEAD
    uint8_t Debug_With_UART(void); 
    //HAL_UART_Transmit(&huart, Debug_With_UART(), 20, HAL_MAX_DELAY);使用HAL_UART_Transmit发送数据,长度依据最后的buffer长度而定

=======
    void Debug_With_UART_Send(UART_HandleTypeDef *huart); 
    
    void Debug_With_UART_Recieve(char rx_buffer[]); 
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
/*************************************************************/
};





#ifdef __cplusplus
}
#endif