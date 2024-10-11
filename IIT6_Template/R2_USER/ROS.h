#pragma once
#include "Global_Namespace.h"
#include "ros_dependence/drive_uart.h"
#include "ros_dependence/data_pool.h"
#include "ros_dependence/tool.h"

using namespace ROS_Namespace;

union ROS_data
{
    float f;
    uint8_t c[4];
}x,y,vx,vy;

typedef struct readFromRos
{
    float x;
    float y;
    float vx;
    float vy;
    uint8_t ctrl_mode;//0x01:position 0x02:velocity
    uint8_t ctrl_flag;//0x01: chassis_ctrl 0x02: gimbal_ctrl
    uint8_t chassis_init;// 0x01: chassis_init
    Robot_Status_t status;
}readFromRos;

typedef uint32_t (*SystemTick_Fun)(void);

#ifdef __cplusplus

class ROS : Tools
{

  private:
    UART_TxMsg TxMsg;
    uint8_t header[2];
    uint8_t tail[2];
    uint8_t lenth=0;

  public:
    ROS()
    {		//包头包尾
      header[0] = 0x55;
      header[1] = 0xAA;
      tail[0] = 0x0D;
      tail[1] = 0x0A;
    }
    void Send_To_ROS(Robot_Twist_t speed);
    int8_t Recieve_From_ROS(uint8_t *buffer);
    readFromRos readFromRosData;
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
    static SystemTick_Fun get_systemTick;
    

};

#endif
