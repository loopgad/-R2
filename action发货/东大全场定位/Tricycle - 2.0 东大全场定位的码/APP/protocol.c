#include "protocol.h"
#include "usart.h"

extern u8 USART_RX_BUF[USART_REC_LEN];
int16_t throttle1,throttle2,throttle3,throttle4;


void Prepare_Data_1(void)
{
	int16_t send_data;
	unsigned char sum = 0;
	
	USART_RX_BUF[0] = 0XAA;
	sum += USART_RX_BUF[0];
	USART_RX_BUF[1] = 0XAA;
	sum += USART_RX_BUF[1];
	
	USART_RX_BUF[2] = 0X01;
	sum += USART_RX_BUF[2];
	
	USART_RX_BUF[3] = 0X0C;
	sum += USART_RX_BUF[3];
	
	send_data = 100;
	USART_RX_BUF[4] = (send_data>>8);
	sum += USART_RX_BUF[4];
	USART_RX_BUF[5] = send_data;
	sum += USART_RX_BUF[5];
	
	send_data = 100;
	USART_RX_BUF[6] = (send_data>>8);
	sum += USART_RX_BUF[6];
	USART_RX_BUF[7] = send_data;
	sum += USART_RX_BUF[7];
	
  send_data = 100;
	USART_RX_BUF[8] = (send_data>>8);
	sum += USART_RX_BUF[8];
	USART_RX_BUF[9] = send_data;
	sum += USART_RX_BUF[9];
	
  send_data = 0;
	USART_RX_BUF[10] = (send_data>>8);
	sum += USART_RX_BUF[10];
	USART_RX_BUF[11] = send_data;
	sum += USART_RX_BUF[11];
	
  send_data = 0;
	USART_RX_BUF[12] = (send_data>>8);
	sum += USART_RX_BUF[12];
	USART_RX_BUF[13] = send_data;
	sum += USART_RX_BUF[13];
	
  send_data = 0;
	USART_RX_BUF[14] = (send_data>>8);
	sum += USART_RX_BUF[14];
	USART_RX_BUF[15] = send_data;
	sum += USART_RX_BUF[15];
	
	USART_RX_BUF[16] = sum;
	
	sum = 0;
	
	USART_RX_BUF[17] = 0XAA;
	sum += USART_RX_BUF[17];
	USART_RX_BUF[18] = 0XAA;
	sum += USART_RX_BUF[18];
	
	USART_RX_BUF[19] = 0X02;
	sum += USART_RX_BUF[19];
	
	USART_RX_BUF[20] = 0X12;
	sum += USART_RX_BUF[20];
	
  send_data = 0;
	USART_RX_BUF[21] = (send_data>>8);
	sum += USART_RX_BUF[21];
	USART_RX_BUF[22] = send_data;
	sum += USART_RX_BUF[22];
	
  send_data = 0;
	USART_RX_BUF[23] = (send_data>>8);
	sum += USART_RX_BUF[23];
	USART_RX_BUF[24] = send_data;
	sum += USART_RX_BUF[24];
	
  send_data = 0;
	USART_RX_BUF[25] = (send_data>>8);
	sum += USART_RX_BUF[25];
	USART_RX_BUF[26] = send_data;
	sum += USART_RX_BUF[26];
	
  send_data = 0;
	USART_RX_BUF[27] = (send_data>>8);
	sum += USART_RX_BUF[27];
	USART_RX_BUF[28] = send_data;
	sum += USART_RX_BUF[28];
	
  send_data = 0;
	USART_RX_BUF[29] = (send_data>>8);
	sum += USART_RX_BUF[29];
	USART_RX_BUF[30] = send_data;
	sum += USART_RX_BUF[30];
	
  send_data = 0;
	USART_RX_BUF[31] = (send_data>>8);
	sum += USART_RX_BUF[31];
	USART_RX_BUF[32] = send_data;
	sum += USART_RX_BUF[32];
	
  send_data = 0;
	USART_RX_BUF[33] = (send_data>>8);
	sum += USART_RX_BUF[33];
	USART_RX_BUF[34] = send_data;
	sum += USART_RX_BUF[34];
	
  send_data = 0;
	USART_RX_BUF[35] = (send_data>>8);
	sum += USART_RX_BUF[35];
	USART_RX_BUF[36] = send_data;
	sum += USART_RX_BUF[36];
	
  send_data = 0;
	USART_RX_BUF[37] = (send_data>>8);
	sum += USART_RX_BUF[37];
	USART_RX_BUF[38] = send_data;
	sum += USART_RX_BUF[38];
	
	USART_RX_BUF[39] = sum;
}
