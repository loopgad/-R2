#pragma once
// 使用 extern "C" 包裹 C 语言的头文件
#ifdef __cplusplus
extern "C" {
#endif

#include "Serial_device.h"
#include "math.h"
#include "stdint.h"

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
// 包含其他头文件，如有必要
// #include "usbd_cdc_if.h"

// 定义 C++ 类
class ros : public SerialDevice 
{
private:
    char header[2] = {0x55, 0xAA};
    char ender[2] = {0x0D, 0x0A};

    union Send {
        float data;
        unsigned char array[4];
    } ToROSworld_px, ToROSworld_py, ToROSworld_vx, ToROSworld_vy;

    union Recieve {
        float data;
        unsigned char array[4];
    } FromROS_NEXTPOINTX, FromROS_NEXTPOINTY;

public:
    ros(UART_HandleTypeDef *huart); // 构造函数
    float nextpoint[2]; // 用于接收下一个目标点
    unsigned char ros_tx_buffer[22]; // 每次 Send_to_ROS 修改完后直接发送这个 buffer
    void Send_to_ROS(float wx, float wy, float vx, float vy);
    void GET_ROS_DATA(uint8_t *byte, float *nextpoint);
    void handleReceiveData(uint8_t byte);
};
#endif

