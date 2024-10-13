#include"Callback_Function.h"

uint8_t RxBuffer_for3[1] = {0};
uint8_t RxBuffer_for2[23] = {0};
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   
	static uint16_t PPM_buf[10]={0};
	static uint8_t ppm_update_flag=0;
    static uint32_t now_ppm_time_send=0;
	static uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;
    static int PPM_Connected_Flag=0;
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//记录脉冲时间
	
	if(GPIO_Pin==GPIO_PIN_7) //判断是不是ppm通道的信息
	{
		PPM_Connected_Flag=1;
		
		last_ppm_time=now_ppm_time;//更新ppm时间节点
		now_ppm_time_send=now_ppm_time=10000*TIME_ISR_CNT+TIM2->CNT;//us
		ppm_time_delta=now_ppm_time-last_ppm_time;//计算脉冲时间
		
		if(ppm_ready==1)//检测PPM信号是否是第一个通道
		{
			
			if(ppm_time_delta>=2200)//对ppm信号进行限幅
			{
				ppm_ready = 1;
				ppm_sample_cnt=0;//设置该信号为第一个
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//PWM在1000-2000us区间内
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//值传递 
				if(ppm_sample_cnt>=8)//只用到0-7�的8个通道，但是总共有10个通道
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));
					//ppm_ready=0;
					ppm_sample_cnt=0;   
				}
			}else{  
				ppm_ready=0;
			}
			
		}else if(ppm_time_delta>=2200)//֡2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
		if(ROCK_L_X>1450&&ROCK_L_X<1550)ROCK_L_X=1500; //添加死区
		if(ROCK_L_Y>1450&&ROCK_L_Y<1550)ROCK_L_Y=1500;
		
		if(ROCK_R_X>1400&&ROCK_R_X<1600)ROCK_R_X=1500;
		if(ROCK_R_Y>1450&&ROCK_R_X<1550)ROCK_R_Y=1500;
		
		if(SWA>900&&SWA<1100)SWA=1000;
		if(SWA>1900&&SWA<2100)SWA=2000;
		
		if(SWD>900&&SWD<1100)SWD=1000;
		if(SWD>1900&&SWD<2100)SWD=2000;
		
		if(SWB>900&&SWB<1100)SWB=1000;
		if(SWB>1450&&SWB<1550)SWB=1500;
		if(SWB>1900&&SWB<2100)SWB=2000;
		
		if(SWC>900&&SWC<1100)SWC=1000;
		if(SWC>1450&&SWC<1550)SWC=1500;
		if(SWC>1900&&SWC<2100)SWC=2000;
	}
}




/***********************************************action************************************/


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

    // 判断是否为USART3
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
void Update_Action(float value[6])
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
