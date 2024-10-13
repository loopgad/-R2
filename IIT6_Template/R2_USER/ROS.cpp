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

SystemTick_Fun ROS::get_systemTick = NULL; // 定义一个系统时钟的函数指针，用于获取系统时间

// 注册获取系统时钟的函数
uint8_t ROS::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL) // 如果传入的函数指针不为空
    {
        ROS::get_systemTick = getTick_fun; // 将传入的函数指针赋值给系统时钟函数指针
        return 1; // 返回成功
    }
    else 
        return 0; // 如果传入的函数指针为空，返回失败
}


/**
 * @brief unpack the data from ROS
 * @param buffer pack that recieved from ROS
 * @return int8_t unpack success return 0, else return 1
 */
inline int8_t ROS::Recieve_From_ROS(uint8_t *buffer)
{
    uint_fast8_t index = 0; // 初始化索引
    float *p = NULL;//未用到控制量
    for(int i = 0; i < 2; i++) // 检查数据包头部
    {
        if(buffer[index++] != header[i]) // 如果头部不匹配
            return 2; // 返回错误
    }

    lenth = buffer[index++]; // 获取数据包长度

    for(int i=0; i<2; i++) // 检查数据包尾部
    {
        if(buffer[4+lenth+i] != tail[i]) // 如果尾部不匹配
            return 1; // 返回错误
    }
    
    for(int i=0; i<4; i++) // 解包x分量
    {
        x.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++) // 解包y分量
    {
        y.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++) // 解包vx分量
    {
        vx.c[i] = buffer[index++];
    }
     for(int i=0; i<4; i++) // 解包vy分量
    {
        vy.c[i] = buffer[index++];
    }

    //8位强转32位,获取相对位置与速度
    Robot_Relative_x = x.f; // x分量传递
    Robot_Relative_y = y.f; // y分量传递
    Robot_Relative_Vx = vx.f; //vx分量传递
    Robot_Relative_Vy = vy.f; //vy分量传递
   
    *p = buffer[index++];
    //CRC校验 
    if(buffer[index++]!=serial_get_crc8_value(buffer, lenth+3)) // 如果CRC校验失败
    {   // 重置赋值量
        Robot_Relative_x = 0; 
        Robot_Relative_x = 0; 
        Robot_Relative_Vx = 0;
        Robot_Relative_Vx = 0;
    }

    return 0; // 返回成功
}