#include "Communication_STM32.h"

const unsigned char serial_header[2] = {0x55,0xaa};
const unsigned char serial_ender[2] = {0x0d,0x0a};

union sendUnion0
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data1_union_instance;

union sendUnion1
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data2_union_instance;

union sendUnion2
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data3_union_instance;

union sendUnion3
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data4_union_instance;

union sendUnion4
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data5_union_instance;

union sendUnion5
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data6_union_instance;

union sendUnion6
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data7_union_instance;

union sendUnion7
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data8_union_instance;

union sendUnion8
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data9_union_instance;

union sendUnion9
{
	int data;
	unsigned char tmp_array[4];
} Stm32ToRos_data10_union_instance;


union receiveUnion0
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data1_union_instance;

union receiveUnion1
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data2_union_instance;

union receiveUnion2
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data3_union_instance;

union receiveUnion3
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data4_union_instance;

union receiveUnion4
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data5_union_instance;

union receiveUnion5
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data6_union_instance;

union receiveUnion6
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data7_union_instance;

union receiveUnion7
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data8_union_instance;

union receiveUnion8
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data9_union_instance;

union receiveUnion9
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data10_union_instance;

void STM32_WRITE_TO_ROS(int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8, int data9, int data10)
{
	unsigned char send_buf[46] = {0};
	unsigned char length = 40;
	
	Stm32ToRos_data1_union_instance.data = data1;
	Stm32ToRos_data2_union_instance.data = data2;
	Stm32ToRos_data3_union_instance.data = data3;
	Stm32ToRos_data4_union_instance.data = data4;
	Stm32ToRos_data5_union_instance.data = data5;
	Stm32ToRos_data6_union_instance.data = data6;
	Stm32ToRos_data7_union_instance.data = data7;
	Stm32ToRos_data8_union_instance.data = data8;
	Stm32ToRos_data9_union_instance.data = data9;
	Stm32ToRos_data10_union_instance.data = data10;
	
	for(int i = 0; i < 2; i++)
	{
		send_buf[i] = serial_header[i];
	}
	send_buf[2] = length;
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 3] = Stm32ToRos_data1_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 7] = Stm32ToRos_data2_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 11] = Stm32ToRos_data3_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 15] = Stm32ToRos_data4_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 19] = Stm32ToRos_data5_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 23] = Stm32ToRos_data6_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 27] = Stm32ToRos_data7_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 31] = Stm32ToRos_data8_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 35] = Stm32ToRos_data9_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 39] = Stm32ToRos_data10_union_instance.tmp_array[i];
	}
	send_buf[43] = serial_get_crc8_value(send_buf, 42);
	for(int i = 0; i < 2; i++)
	{
		send_buf[i + 44] = serial_ender[i];
	}
	
	for(int i = 0; i < 46; i++)
	{
		while((USART2->SR&0x40)==0);
		USART2->DR = send_buf[i];
	}
}
unsigned char receive_buf[920] = {0};
unsigned char check_header_buf[2] = {0};
unsigned char USART_Receiver2 = 0x00;
int STM32_READ_FROM_ROS(float *data1, float *data2, float *data3, float *data4, float *data5, float *data6, float *data7, float *data8, float *data9, float *data10)
{
	static short index = 0;
	static unsigned char flag = 0x00;
	
	HAL_UART_Receive_IT(&huart2, &USART_Receiver2, 1);
	
	switch(flag)
	{
		case CHECK_HEADER_FLAG:
		{
			for(int i = 0; i < 1; i++)
			{
				check_header_buf[i] = check_header_buf[i+1];
			}
			check_header_buf[1] = USART_Receiver2;
		
			unsigned char cnt = 0;
			for(int i = 0; i < 2; i++)
			{
				if(check_header_buf[i] == serial_header[i])
				{
					cnt++;
				}
			}
			if(cnt ==2)
			{
				while(index < 2)
				{
					receive_buf[index] = check_header_buf[index];
					index++;
				}
				flag = RECEIVE_DATA_FLAG;
			}
			break;
		}
		case RECEIVE_DATA_FLAG:
		{
			receive_buf[index] = USART_Receiver2;
			index++;
			
			if(index == 43)
			{
				flag = CHECK_VALUE_FLAG;
			}
			break;
		}
		case CHECK_VALUE_FLAG:
		{
			unsigned char check_value = serial_get_crc8_value(receive_buf, index - 1);
			if(check_value == USART_Receiver2)
			{
				receive_buf[index] = USART_Receiver2;
				index++;
				flag = FINAL_RECEIVE_FLAG;
			}
			else
			{
				index = 0;
				flag = CHECK_HEADER_FLAG;
			}
			break;
		}
		case FINAL_RECEIVE_FLAG:
		{
			receive_buf[index] = USART_Receiver2;
			index++;
			
			if(index == 46)
			{
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data1_union_instance.tmp_array[i] = receive_buf[i + 3];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data2_union_instance.tmp_array[i] = receive_buf[i + 7];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data3_union_instance.tmp_array[i] = receive_buf[i + 11];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data4_union_instance.tmp_array[i] = receive_buf[i + 15];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data5_union_instance.tmp_array[i] = receive_buf[i + 19];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data6_union_instance.tmp_array[i] = receive_buf[i + 23];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data7_union_instance.tmp_array[i] = receive_buf[i + 27];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data8_union_instance.tmp_array[i] = receive_buf[i + 31];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data9_union_instance.tmp_array[i] = receive_buf[i + 35];
				}
				for(int i = 0; i < 4; i++)
				{
					RosToStm32_data10_union_instance.tmp_array[i] = receive_buf[i + 39];
				}
		
				*data1 = RosToStm32_data1_union_instance.data;
				*data2 = RosToStm32_data2_union_instance.data;
				*data3 = RosToStm32_data3_union_instance.data;
				*data4 = RosToStm32_data4_union_instance.data;
				*data5 = RosToStm32_data5_union_instance.data;
				*data6 = RosToStm32_data6_union_instance.data;
				*data7 = RosToStm32_data7_union_instance.data;
				*data8 = RosToStm32_data8_union_instance.data;
				*data9 = RosToStm32_data9_union_instance.data;
				*data10 = RosToStm32_data10_union_instance.data;
				index = 0;
				flag = CHECK_HEADER_FLAG;
				return 1;
			}
			break;
		}
	}
	return 0;
}
unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len)
{
	unsigned char crc = 0;
	unsigned char i;
	while(len--)
	{
		crc ^= *data++;
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
