/**
 * @file ROS.cpp
 * @brief 上下位机的通信文件，包括数据的打包和解包，数据的发送和接收。使用串口DMA
 * @version 0.1
 * @date 2024-04-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "ROS.h"

SystemTick_Fun ROS::get_systemTick = NULL;

union ROS_data
{
    float f;
    uint8_t c[4];
}x,y,z;


uint8_t ROS::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        ROS::get_systemTick = getTick_fun;
        return 1;
    }
    else 
        return 0;
}


/**
 * @brief stm32 send data to ROS
 * @note 该函数用于将数据打包发送给ROS, 通过轮子速度解算后的速度转化为了mm/s和mRad/s
 */
void ROS::Send_To_ROS(Robot_Twist_t speed)
{
    uint8_t buffer[6+6];
    int index = 0;
    buffer[index++] = header[0];
    buffer[index++] = header[1];
    buffer[index++] = 1;
    _tool_buffer_append_int16(buffer, (int16_t)(speed.linear.x*1000), &index);
    _tool_buffer_append_int16(buffer, (int16_t)(speed.linear.y*1000), &index);
    _tool_buffer_append_int16(buffer, (int16_t)(speed.angular.z*1000), &index);
    buffer[index++] = serial_get_crc8_value(buffer, 4);
    buffer[index++] = tail[0];
    buffer[index++] = tail[1];
}


/**
 * @brief upack the data from ROS
 * @param buffer pack that recieved from ROS
 * @return int8_t unpack success return 0, else return 1
 */
int8_t ROS::Recieve_From_ROS(uint8_t *buffer)
{
    int index = 0;
    for(int i = 0; i < 2; i++)
    {
        if(buffer[index++] != header[i])
            return 2;
    }

    lenth = buffer[index++];

    for(int i=0; i<2; i++)
    {
        if(buffer[4+lenth+i] != tail[i])
            return 1;
    }
    
    for(int i=0; i<4; i++)
    {
        x.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++)
    {
        y.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++)
    {
        z.c[i] = buffer[index++];
    }
    //8位强转32位
    readFromRosData.x = x.f;
    readFromRosData.y = y.f;
    readFromRosData.z = z.f;
		//控制状态
    readFromRosData.ctrl_mode = buffer[index++];
    readFromRosData.ctrl_flag = buffer[index++];
		//底盘状态赋值
    readFromRosData.chassis_init = buffer[index++];
    readFromRosData.status.robot_init = (PLAYLIST)buffer[index++];
    readFromRosData.status.path_mode = (PLAYLIST)buffer[index++];
    readFromRosData.status.sensor = (PLAYLIST)buffer[index++];
    readFromRosData.status.control_mode = (PLAYLIST)buffer[index++];

		//CRC校验 
    if(buffer[index++]!=serial_get_crc8_value(buffer, lenth+3))
    {
        readFromRosData.x = 0;
        readFromRosData.y = 0;
        readFromRosData.z = 0;
        readFromRosData.ctrl_mode = NORMAL;
        readFromRosData.ctrl_flag = 0;
        readFromRosData.chassis_init = false;
    }

    return 0;
}
