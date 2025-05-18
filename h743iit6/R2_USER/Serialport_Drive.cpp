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


#include "Serialport_Drive.h"
#include <cstdint>


using namespace ROS_Namespace;


/***********************************ROS部分***************************************/

unsigned char Serialport_Drive::ros_serial_get_crc8_value(unsigned char *tem_array, unsigned char len)
{
    unsigned char crc = 0;
    unsigned char i;
    while(len--)
    {
        crc ^= *tem_array++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}


void Serialport_Drive::Send_to_ROS(float wx, float wy, float vx, float vy,  UART_HandleTypeDef *huart)
{
    unsigned char ros_tx_buffer[22]; // 每次 Send_to_ROS 修改完后直接发送这个 buffer
	int index=0;
    int length = 16;
	ToROSworld_px.data=wx/1000;
	ToROSworld_py.data=wy/1000;
	ToROSworld_vx.data=vx/1000;
	ToROSworld_vy.data=vy/1000;
	ros_tx_buffer[0] = header[0];
    ros_tx_buffer[1] = header[1];
    ros_tx_buffer[2] = length;
    for (int i=0;i<4;i++)
	{
		ros_tx_buffer[i+3]= ToROSworld_px.array[i];
		ros_tx_buffer[i+7]= ToROSworld_py.array[i];
		ros_tx_buffer[i+11]=ToROSworld_vx.array[i];
		ros_tx_buffer[i+15]=ToROSworld_vy.array[i];
	}
    ros_tx_buffer[19] = ros_serial_get_crc8_value(ros_tx_buffer, length+1);
    ros_tx_buffer[20] = ender[0];
    ros_tx_buffer[21] = ender[1];
	HAL_UART_Transmit(huart, ros_tx_buffer, sizeof(ros_tx_buffer),HAL_MAX_DELAY);
}

void Serialport_Drive::GET_ROS_DATA(uint8_t buffer[])
{
	static int index = 0;
	static unsigned char length = 0;
	static bool header_received = false;

 // 判断包头是否已经收到
 if (!header_received) {
     if (index == 0 && buffer[0] == header[0]) {
     }
     else if (index == 1 && buffer[1] == header[1]) {
         header_received = true;
     }
     else {
         index = 0;
     }
     return;
 }

 // 包头已收到，继续自增索引
        index++;
 // 当接收到长度字节时，更新包长
 if (index == 3) {
     length = buffer[2];
 }

 // 检查是否接收完整个包
 if (index == length + 4) {
     unsigned char check_value = ros_serial_get_crc8_value(&buffer[11], length + 1);
     if (check_value == buffer[length + 3]) { // 校验通过
         for (int i = 0; i < 4; i++) {
             FromROS_NEXTPOINTX.array[i] = buffer[i + 3];
             FromROS_NEXTPOINTY.array[i] = buffer[i + 7];
         }
         nextpoint[0] = FromROS_NEXTPOINTX.data;
         nextpoint[1] = FromROS_NEXTPOINTY.data;

     }
     else {
     }

     // 重置状态以接收下一个数据包
     index = 0;
     header_received = false;
 }
}

/******************************************************************************/



/****************************Xbox部分************************************/
/***********************************************xbox信息处理函数************************************/
void Serialport_Drive::Xbox_Receive_Data(uint8_t byte)
{
    switch (state_)
    {
    case WAITING_FOR_HEADER_0:
        if (byte == FRAME_HEAD_0_RC9)
        {
            state_ = WAITING_FOR_HEADER_1;
            rx_frame_mat.frame_head[0] = byte; // 存储帧头
        }
        break;
    case WAITING_FOR_HEADER_1:
        if (byte == FRAME_HEAD_1_RC9)
        {
            state_ = WAITING_FOR_ID;
            rx_frame_mat.frame_head[1] = byte; // 存储帧头
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0; // 重置状态机
        }
        break;
    case WAITING_FOR_ID:
        rx_frame_mat.frame_id = byte; // 存储帧ID
        state_ = WAITING_FOR_LENGTH;
        break;
    case WAITING_FOR_LENGTH:
        rx_frame_mat.data_length = byte; // 存储数据长度
        rxIndex_ = 0;                    // 初始化索引
        state_ = WAITING_FOR_DATA;
        break;
    case WAITING_FOR_DATA:
        rx_frame_mat.rx_temp_data_mat[rxIndex_++] = byte; // 存储接收到的数据
        if (rxIndex_ >= rx_frame_mat.data_length)
        {
            state_ = WAITING_FOR_CRC_0;
        }
        break;
    case WAITING_FOR_CRC_0:
        rx_frame_mat.check_code.crc_buff[0] = byte; // 存储 CRC 校验的高字节
        state_ = WAITING_FOR_CRC_1;
        break;
    case WAITING_FOR_CRC_1:
        rx_frame_mat.check_code.crc_buff[1] = byte; // 存储 CRC 校验的低字节
        state_ = WAITING_FOR_END_0;
        break;
    case WAITING_FOR_END_0:
        if (byte == FRAME_END_0_RC9)
        {
            state_ = WAITING_FOR_END_1;
            rx_frame_mat.frame_end[0] = byte; // 存储帧尾
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0; // 重置状态机
        }
        break;
    case WAITING_FOR_END_1:
        if (byte == FRAME_END_1_RC9)
        {
            rx_frame_mat.frame_end[1] = byte; // 存储帧尾        
                // CRC 校验成功，将数据复制到最终数组
                for (uint8_t i = 0; i < rx_frame_mat.data_length; i++)
                {
                    Xbox_Namespace::xbox_raw_data[i] = rx_frame_mat.rx_temp_data_mat[i];
                }
            //}

            // 重置状态机
            state_ = WAITING_FOR_HEADER_0;
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0; // 重置状态机
        }
        break;
    default:
        state_ = WAITING_FOR_HEADER_0;
        break;
    }
}

/***********************************Action信息处理函数***************************************/
void Serialport_Drive::Action_Receive_Data(uint8_t RxBuffer[])
{
		ch = RxBuffer[0];
		
		//ch = huart->Instance->DR & 0xFF;

        switch (count) {
            case 0:
			{
                if (ch == 0x0d)
                    count++;
                else
                    count = 0;
            }
			break;
			
            case 1:
			{
                if (ch == 0x0a) {
                    i = 0;
                    count++;
                }
				else if (ch == 0x0d)
                    count = 6; // 判断是否为帧尾
                else
                    count = 0;
			}
			break;
			
            case 2:
			{
                posture.data[i] = ch;
                i++;
                if (i >= 24) {
                    i = 0;
                    count++;
                }
            }
			break;
			
            case 3:
			{
                if (ch == 0x0a)
                    count++;
                else
                    count = 0;
			}
			break;
			
            case 4:
			{
                if (ch == 0x0d) {
                     action_Data[0]= posture.ActVal[0];   //zangle
                     action_Data[1]= posture.ActVal[1];   //xangle
                     action_Data[2]= posture.ActVal[2];   //yangle
                     action_Data[3]= posture.ActVal[3];   //pos_x 
                     action_Data[4]= posture.ActVal[4];   //pos_y 
                     action_Data[5]= posture.ActVal[5];   //w_z   
					 Update_Action(posture.ActVal);
                }
                count = 0;
			}
			break;
			
            default:
			{
                count = 0;
			}
			break;
        }
		
		RxBuffer[0]=0;
}

void Serialport_Drive::Update_Action(float value[6])
{
	
//update data
	Action_Namespace::ACTION_GL_POS_DATA.LAST_POS_X = Action_Namespace::ACTION_GL_POS_DATA.POS_X;
	Action_Namespace::ACTION_GL_POS_DATA.LAST_POS_Y = Action_Namespace::ACTION_GL_POS_DATA.POS_Y;
	Action_Namespace::ACTION_GL_POS_DATA.LAST_YAW = Action_Namespace::ACTION_GL_POS_DATA.YAW;
// update angleֵ
	Action_Namespace::ACTION_GL_POS_DATA.YAW = value[0]; // 角度值为-180~180
	Action_Namespace::ACTION_GL_POS_DATA.POS_X = value[3]; 
	Action_Namespace::ACTION_GL_POS_DATA.POS_Y = value[4]; 
	Action_Namespace::ACTION_GL_POS_DATA.W_Z = value[5];
//	(Action_Namespace::ACTION_GL_POS_DATA.W_Z)*PI/180

	Action_Namespace::ACTION_GL_POS_DATA.DELTA_POS_X = Action_Namespace::ACTION_GL_POS_DATA.POS_X - Action_Namespace::ACTION_GL_POS_DATA.LAST_POS_X;
	Action_Namespace::ACTION_GL_POS_DATA.DELTA_POS_Y = Action_Namespace::ACTION_GL_POS_DATA.POS_Y - Action_Namespace::ACTION_GL_POS_DATA.LAST_POS_Y;
	Action_Namespace::ACTION_GL_POS_DATA.DELTA_YAW = Action_Namespace::ACTION_GL_POS_DATA.YAW - Action_Namespace::ACTION_GL_POS_DATA.LAST_YAW;
	
	Action_Namespace::ACTION_GL_POS_DATA.REAL_X += (-Action_Namespace::ACTION_GL_POS_DATA.DELTA_POS_X);
	Action_Namespace::ACTION_GL_POS_DATA.REAL_Y += (-Action_Namespace::ACTION_GL_POS_DATA.DELTA_POS_Y);
	Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW += (-Action_Namespace::ACTION_GL_POS_DATA.DELTA_YAW);

//没用到
//	ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_X = (Action_Namespace::ACTION_GL_POS_DATA.REAL_X - 128.901f*sin(ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_YAW * PI / 180+0.30688)) * 0.001;
//	ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_Y = (Action_Namespace::ACTION_GL_POS_DATA.REAL_Y - 128.901f*cos(ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_YAW * PI / 180+0.30688)) * 0.001;	 
}
/***************************************************************************************/

/****************************VOFA调试函数*************************************/
void Serialport_Drive::Debug_With_UART_Send(UART_HandleTypeDef *huart){
    Debug_Data.fdata[0] = 12.2f;
    Debug_Data.fdata[1] = 15.3f;
    uint8_t buffer[sizeof(Debug_Data.fdata) + sizeof(Debug_Data.tail)]; // 将float数组转换为字节数组，因为HAL_UART_Transmit发送的是字节
    memcpy(buffer, (Debug_Data.fdata), sizeof(Debug_Data.fdata));   // 复制float数据到buffer
	
    memcpy(buffer + sizeof(Debug_Data.fdata), Debug_Data.tail, sizeof(Debug_Data.tail));   // 复制尾部数据到buffer
	
    HAL_UART_Transmit(huart, buffer, sizeof(buffer), HAL_MAX_DELAY);//使用HAL_UART_Transmit发送数据,长度依据最后的buffer长度而定 
}

float data_kp = 0;
float data_ki = 0;
float data_kd = 0;
float data_output = 0;

void Serialport_Drive::Debug_With_UART_Recieve(char rx_buffer[]){
// 假设收到的数据包
    /*
    const char *packet = "<STX>,123,456,789,4000,<ETX>";
    */
    // 使用 sscanf 解析数据包，并在格式字符串中直接指定包头和包尾
    sscanf(rx_buffer, "<STX>,%f,%f,%f,%f,<ETX>", &data_kp, &data_ki, &data_kd, &data_output);
}

/**********************************************************/