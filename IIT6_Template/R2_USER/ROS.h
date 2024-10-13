#pragma once
#include "Global_Namespace.h"
#include "drive_uart.h"
#include "tool.h"

using namespace ROS_Namespace;

union ROS_data
{
    float f;
    uint8_t c[4];
}x,y,vx,vy;


typedef uint32_t (*SystemTick_Fun)(void);

#ifdef __cplusplus

class ROS : Tools
{

  private:
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
    inline int8_t Recieve_From_ROS(uint8_t *buffer);
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
    static SystemTick_Fun get_systemTick;
    

};

#endif
