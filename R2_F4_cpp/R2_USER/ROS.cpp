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


#ifdef __cplusplus
extern "C" {
#endif

using namespace ROS_Namespace;


/**
 * @brief unpack the data from ROS
 * @param buffer pack that recieved from ROS
 * @return int8_t unpack success return 0, else return 1
 */
int8_t ROS::Recieve_From_ROS(uint8_t *buffer)
{
    uint_fast8_t index = 0; // 初始化索引
    float *p = nullptr;//未用到控制量
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
   
	if (buffer[index++] != CRC8_Table(buffer, lenth + 3)) // 如果CRC校验失败
	{
    // 重置赋值量
    Robot_Relative_x = 0; 
    Robot_Relative_y = 0; 
    Robot_Relative_Vx = 0;
    Robot_Relative_Vy = 0;
	}

    return 0; // 返回成功
}

#ifdef __cplusplus
}
#endif
