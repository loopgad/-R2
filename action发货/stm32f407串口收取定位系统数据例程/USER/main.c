#include "all.h"

int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(84);
	
	uart_init(115200);
	uart3_init(115200);

  while(1)
	{
		delay_ms(50);
		
		printf("Yaw:%f\tX:%f\tY:%f\r\n", zangle, pos_x, pos_y);
	}
}
