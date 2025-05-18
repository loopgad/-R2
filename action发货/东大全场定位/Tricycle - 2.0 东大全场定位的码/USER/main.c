#include "sys.h"
#include "delay.h"
#include "timer2.h"
#include "task.h"
#include "control.h"


int main(void)
{ 	
	delay_init(168);  //初始化延时函数
	BSP_Init();
	
	while(1)
	{
		if(Count_1ms>=1)
		{	
			Count_1ms = 0;
			Task_1000HZ();
		}
		if(Count_2ms>=2)
		{
			Count_2ms = 0;
			Task_500HZ();
		}
		if(Count_4ms>=4)
		{
			Count_4ms = 0;
			Task_250HZ();
		}
		if(Count_5ms>=5)
		{
			Count_5ms = 0;
			Task_200HZ();
		}
		if(Count_10ms>=10)
		{
			Count_10ms = 0;
			Task_100HZ();
		}
		if(Count_20ms>=20)
		{
			Count_20ms = 0;
			Task_50HZ();
		}
		if(Count_50ms>=50)
		{
			Count_50ms = 0;
			Task_20HZ();
		}
		if(Count_100ms>=100)
		{
			Count_100ms = 0;
			Task_10HZ();
		}
		if(Count_1000ms>=1000)
		{
			Count_1000ms = 0;
			Task_1HZ();
		}
	}
}
