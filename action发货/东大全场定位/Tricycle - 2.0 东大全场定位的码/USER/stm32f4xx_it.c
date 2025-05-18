#include "stm32f4xx_it.h"

uint8_t Count_1ms,Count_2ms,Count_4ms,Count_5ms,Count_10ms,Count_20ms,Count_50ms,Count_100ms;
int Count_1000ms;
extern uint8_t Bsp_Int_Ok;
extern uint32_t Timer2_Count;

void TIM2_IRQHandler(void)//Timer3�ж�
{	
	if(TIM2->SR & TIM_IT_Update)
	{     
		TIM2->SR = ~TIM_FLAG_Update;//����жϱ�־
		
		if( Bsp_Int_Ok == 0 )	return;//Ӳ��δ��ʼ����ɣ��򷵻�
		Timer2_Count++;
		Count_1ms++;
		Count_2ms++;
		Count_4ms++;
		Count_5ms++;
		Count_10ms++;
		Count_20ms++;
		Count_50ms++;
		Count_100ms++;
		Count_1000ms++;
	}
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		//PWM_control_signal_pull_down_all();
  }
}

