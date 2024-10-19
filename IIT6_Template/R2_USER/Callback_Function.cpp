#include"Callback_Function.h"

uint8_t RxBuffer_for3[1] = {0};
uint8_t RxBuffer_for2[23] = {0};

/***********************************************xbox信息处理函数************************************/
inline void Handle_Receive_Data(uint8_t byte)
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

            // 计算 CRC 校验值并与接收到的 CRC 进行比较
            rx_frame_mat.crc_calculated = CRC16_Table(rx_frame_mat.rx_temp_data_mat, rx_frame_mat.data_length);
            if (rx_frame_mat.crc_calculated == rx_frame_mat.check_code.crc_code)
            {
                // CRC 校验成功，将数据复制到最终数组
                for (uint8_t i = 0; i < rx_frame_mat.data_length; i++)
                {
                    xbox_received_data[i] = rx_frame_mat.rx_temp_data_mat[i];
                }
            }

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

/***********************************************串口外设************************************/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static ROS ROS;
	static uint8_t buffer_tmp[24] = {0}; //只用到前23
	static uint_fast8_t index = 0;

    static union {
        uint8_t data[24];
        float ActVal[6];
    } posture;
		
	static uint8_t ch;
    static uint8_t count = 0;
    static uint8_t i = 0;

	// USART1接收xbox手柄信息
	if (huart==&huart1) {
		
		Handle_Receive_Data(USART1->DR);

	}

    // 判断是否为USART3,接收action数据
    if (huart==&huart3) {
		
		ch = RxBuffer_for3[0];
		
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
		
		RxBuffer_for3[0]=0;
        //重新启动USART3接收中断
		HAL_UART_Receive_IT(&huart3,RxBuffer_for3, 1);
		
    }

		// 判断是否为USART2
    if (huart==&huart2) {
    	// 使用位操作将32位数据拷贝到buffer_tmp数组中
    	{
			uint32_t tmp = USART2->RDR;
    		buffer_tmp[index++] = (tmp >> 24) & 0xFF; // 高8位
    		buffer_tmp[index++] = (tmp >> 16) & 0xFF; // 中高8位
    		buffer_tmp[index++] = (tmp >> 8) & 0xFF;  // 中低8位
    		buffer_tmp[index++] = tmp & 0xFF;        // 低8位
    	}

		if(index > 23)
    	ROS.Recieve_From_ROS(buffer_tmp);
		//重新启动USART2接收中断
		HAL_UART_Receive_IT(&huart2,RxBuffer_for3, 1);
	}
}

//action数据更新
inline void Update_Action(float value[6])
{
	
//update data
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
	ACTION_GL_POS_DATA.LAST_YAW = ACTION_GL_POS_DATA.YAW;
// update angleֵ
	ACTION_GL_POS_DATA.YAW = value[0]; // 角度值为-180~180
	ACTION_GL_POS_DATA.POS_X = value[3]; 
	ACTION_GL_POS_DATA.POS_Y = value[4]; 
	ACTION_GL_POS_DATA.W_Z = value[5];
//	(ACTION_GL_POS_DATA.W_Z)*PI/180

	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
	ACTION_GL_POS_DATA.DELTA_YAW = ACTION_GL_POS_DATA.YAW - ACTION_GL_POS_DATA.LAST_YAW;
	
	ACTION_GL_POS_DATA.REAL_X += (-ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (-ACTION_GL_POS_DATA.DELTA_POS_Y);
	ACTION_GL_POS_DATA.REAL_YAW += (-ACTION_GL_POS_DATA.DELTA_YAW);
	
	ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_X = (ACTION_GL_POS_DATA.REAL_X - 128.901f*sin(ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_YAW * PI / 180+0.30688)) * 0.001;
	ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_Y = (ACTION_GL_POS_DATA.REAL_Y - 128.901f*cos(ROBOT_Namespace::ROBOT_REAL_POS_DATA.POS_YAW * PI / 180+0.30688)) * 0.001;	 
}
