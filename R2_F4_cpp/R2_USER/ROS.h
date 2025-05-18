#pragma once
#include "Global_Namespace.h"
#include "tool.h"
#include "crc_util.h"


#ifdef __cplusplus
extern "C" {
#endif


class ROS
{

  private:
		union ROS_data
	{
		float f;
		uint8_t c[4];
	}x,y,vx,vy;
	
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
    int8_t Recieve_From_ROS(uint8_t *buffer);

    

};

#ifdef __cplusplus
}
#endif



