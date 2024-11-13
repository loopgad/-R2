#include "ROS.h"


unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len)
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

ros::ros(UART_HandleTypeDef *huart) : SerialDevice(huart){
}
	

void ros::Send_to_ROS(float wx,float wy,float vx,float vy)
{
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
    ros_tx_buffer[19] = serial_get_crc8_value(ros_tx_buffer, length+1);
    ros_tx_buffer[20] = ender[0];
    ros_tx_buffer[21] = ender[1];
	HAL_UART_Transmit(huart_,ros_tx_buffer,sizeof(ros_tx_buffer),HAL_MAX_DELAY);
}

void ros::GET_ROS_DATA(uint8_t *byte, float *nextpoint)
{
    static unsigned char buffer[12];
	static int index = 0;
	static unsigned char length = 0;
	static bool header_received = false;

 // 判断包头是否已经收到
 if (!header_received) {
     if (index == 0 && *byte == header[0]) {
         buffer[index++] = *byte;
     }
     else if (index == 1 && *byte == header[1]) {
         buffer[index++] = *byte;
         header_received = true;
     }
     else {
         index = 0;
     }
     return;
 }

 // 包头已收到，继续接收数据
 buffer[index++] = *byte;

 // 当接收到长度字节时，更新包长
 if (index == 3) {
     length = buffer[2];
 }

 // 检查是否接收完整个包
 if (index == length + 4) {
     unsigned char check_value = serial_get_crc8_value(buffer, length + 1);
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


void ros::handleReceiveData(uint8_t byte){
	GET_ROS_DATA(&byte,nextpoint); // 处理好接收的ROS目标点数据
}
