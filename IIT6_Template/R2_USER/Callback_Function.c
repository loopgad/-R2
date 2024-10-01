#include"Callback_Function.h"



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   
	static uint8_t ppm_update_flag=0;
    static uint32_t now_ppm_time_send=0;
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
		
		if(ppm_ready==1)//�ж�֡����ʱ����ʼ�����µ�һ��PPM
		{
			
			if(ppm_time_delta>=2200)//֡������ƽ����2ms=2000us�����ڲ����ϰ汾ң������//���ջ����PPM�źŲ���׼�������ֽ����쳣ʱ�����Ը�С��ֵ�������������һ����ʹ����ط��ϰ汾ң����
			{
				ppm_ready = 1;
				ppm_sample_cnt=0;//��Ӧ��ͨ��ֵ
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//����PWM������1000-2000us
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//��Ӧͨ��д�뻺���� 
				if(ppm_sample_cnt>=8)//���ν�������0-7��ʾ8��ͨ���������������ʾ10��ͨ���������ֵӦ��Ϊ0-9�������޸�
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
		if(ROCK_L_X>1450&&ROCK_L_X<1550)ROCK_L_X=1500; //��������
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
	
    static union {
        uint8_t data[24];
        float ActVal[6];
    } posture;
		
	static uint8_t ch;
    static uint8_t count = 0;
    static uint8_t i = 0;

    // ����Ƿ���USART3���ж�
    if (huart==&huart3) {
        // ��ȡ���յ�������
		
		ch = RxBuffer_for3[0];
		
		//ch = huart->Instance->DR & 0xFF;

        // �������յ�������
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
                    count = 6; // �������0x0dֱ�ӽ���Ĭ��case
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
		
        // ����ʹ�ܴ��ڽ����ж�
		HAL_UART_Receive_IT(&huart3,RxBuffer_for3, 1);
    }
}
