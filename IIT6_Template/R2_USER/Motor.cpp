#include "Motor.h"


//Motor m3508[3];  创建电机类实例

/**
  * @brief  电机组重初始化
	* @param  None
	* @retval None
  * @attention  在main调用一下初始化
  *电机初始化（包含PID，电机类型）
  */
void  Motor_DeInit(Motor Motor[], uint_fast8_t sizeof_Motor)
{
	//PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone
	
	//速度环
    for(int i = 0; i < sizeof_Motor; i++){
        Motor[i].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
	    //m3508[1].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
	    //m3508[2].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);

	//位置环pid
	    Motor[i].Motor_PID_POS.PID_Parameter_Deinit(100, 0, 1, 7000, 7000, 0.05);
	    //m3508[1].Motor_PID_POS.PID_Parameter_Deinit(100, 0, 1, 7000, 7000, 0.05);
	    //m3508[2].Motor_PID_POS.PID_Parameter_Deinit(100, 0, 1, 7000, 7000, 0.05);
    }
	

//	//电机类型
//	MOTOR_REAL_INFO[0].type = RM_3508;
//	MOTOR_REAL_INFO[1].type = RM_3508;
//	MOTOR_REAL_INFO[2].type = RM_3508;
	
	//电机模式
	MOTOR_REAL_INFO[0].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[1].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[2].unitMode = SPEED_CONTROL_MODE;

}

void Motor_Manager::Motor_control(void){
	name = "Motor_control";
	MotorrCtrl();
}

/**
  * @brief  MotorrCtrl电机控制
	* @param  None
	* @retval None
  */
void Motor_Manager::MotorrCtrl(void)
{	
	for(int i = 0; i < 3; i++)
	{		
		switch(MOTOR_REAL_INFO[i].unitMode)
		{
		case SPEED_CONTROL_MODE://速度模式
		m3508[i].Motor_PID_RPM.PID_incremental_PID_calculation(MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);//速度环
		MOTOR_REAL_INFO[i].TARGET_CURRENT =m3508[i].Motor_PID_RPM.output;//电流赋值
			break;
			
		default:
			break;
		}

	}
	MotorCURRENT_CAN_send();
}

// 利用电机通过CAN反馈的数据更新m3508的状态信息       //public
// 接受频率：1kHz
void Motor_Manager::Motor_CAN_update(CAN_RxHeaderTypeDef *msg, uint8_t can1_RxData[8])
{
	switch(msg -> StdId)  // 检测标准ID
	{
    case MotorR_ID_1:
		{ 
			MOTOR_REAL_INFO[0].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
		case MotorR_ID_2:
		{ 
			MOTOR_REAL_INFO[1].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
		case MotorR_ID_3:
		{ 
			MOTOR_REAL_INFO[2].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			MOTOR_REAL_INFO[2].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;	
		
		default: 
			 break;
	}
}

//发送电流
void Motor_Task::MotorCURRENT_CAN_send(void)
{
	/***********************************用于ID为 1 2 3 4 的电机*********************************/
	CAN_TxHeaderTypeDef tx_message_1;
  uint8_t send_buf1[8] = {0};
	uint32_t msg_box1;
	// 配置控制段
	tx_message_1.IDE = CAN_ID_STD;//报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	tx_message_1.RTR = CAN_RTR_DATA;//报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	tx_message_1.DLC = 0x08;//数据段长度
	tx_message_1.TransmitGlobalTime = DISABLE;
	// 配置仲裁段和数据段	
	tx_message_1.StdId = 0x200;  // 用于ID为 1 2 3 4 的电机
	
	send_buf1[0] = (uint8_t)(MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	send_buf1[1] = (uint8_t) MOTOR_REAL_INFO[0].TARGET_CURRENT;
	send_buf1[2] = (uint8_t)(MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	send_buf1[3] = (uint8_t) MOTOR_REAL_INFO[1].TARGET_CURRENT;
	send_buf1[4] = (uint8_t)(MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	send_buf1[5] = (uint8_t) MOTOR_REAL_INFO[2].TARGET_CURRENT;
	send_buf1[6] = (uint8_t)(MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	send_buf1[7] = (uint8_t) MOTOR_REAL_INFO[3].TARGET_CURRENT;

	                                                                                                                                                                                          
	    if (HAL_CAN_AddTxMessage(&hcan1,&tx_message_1,send_buf1,&msg_box1)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }		
}

