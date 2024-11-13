/*
Copyright (c) 2024 loopgad 9th_R2_Member

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



