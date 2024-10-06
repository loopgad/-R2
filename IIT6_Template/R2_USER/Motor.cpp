#include "Motor.h"

inline void Motor::Motor_PID_Init(void){
    PID_POS.PID_Parameter_Deinit(100, 0, 1, 7000, 7000, 0.05);
    PID_PRM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
    PID_YAW.PID_Parameter_Deinit(0.1f,0.01f,0.02f, 3.0f, 0.0f, 1.0f);
}

Motor::Motor(int ID_NUM) : PID_POS(MOTOR_PID[0]), PID_PRM(MOTOR_PID[1]), PID_YAW(MOTOR_PID[2]){
    Motor::Motor_PID_Init();
    ID = ID_NUM;
}

void  Motor::MotorCtrl()
{
    static UnitMode CURRENT_UNIT_MODE = SPEED_CONTROL_MODE;
	
    if constexpr (CURRENT_UNIT_MODE == POSITION_CONTROL_MODE) {
        // 位置模式
        PID_POS.PID_incremental_PID_calculation( MOTOR_REAL_INFO[ID].RPM, MOTOR_PID_POS[ID].output); // 速度环
        PID_POS.PID_position_PID_calculation(MOTOR_REAL_INFO[ID].REAL_ANGLE, MOTOR_REAL_INFO[ID].TARGET_POS); // 位置环
    } 
    else if constexpr (CURRENT_UNIT_MODE == SPEED_CONTROL_MODE) {
        // 速度模式
        PID_PRM.PID_incremental_PID_calculation(MOTOR_REAL_INFO[ID].RPM, MOTOR_REAL_INFO[ID].TARGET_RPM); // 速度环
        MOTOR_REAL_INFO[ID].TARGET_CURRENT = MOTOR_PID_RPM[ID].output; // 电流赋值
    } 
    else if constexpr (CURRENT_UNIT_MODE == CURRENT_MODE) {
        // 电流模式，什么都不执行，直接电流赋值
    } 
    else if constexpr (CURRENT_UNIT_MODE == MOTO_OFF) {
        // 关闭电机
        MOTOR_REAL_INFO[ID].TARGET_CURRENT = 0.0f; // 电流赋值
    }
}





//发送电流
void Send_Motor_Currents(void)
{
	
	/***********************************用于ID为 1 2 3 4 的电机*********************************/
	CAN_TxHeaderTypeDef tx_message_1;
    uint8_t send_buf1[8] = {0};
	uint32_t msg_box1;
	// 配置控制段
	tx_message_1.IDE = CAN_ID_STD;  // 报文的11位标准标识符 CAN_ID_STD 表示本报文是标准帧
	tx_message_1.RTR = CAN_RTR_DATA;  // 报文类型标志 RTR位 CAN_ID_STD 表示本报文的数据帧
	tx_message_1.DLC = 0x08;  // 数据段长度
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

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_message_1, send_buf1, &msg_box1) != HAL_OK) {
        // Failed to add message to the transmit mailbox
    }		
}

