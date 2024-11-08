#include "Motor.h"

using namespace Motor_Namespace;



void Motor_Init(Motor_Manager* core,uint_fast8_t sizeof_Motor)
{

    for (int i = 0; i < sizeof_Motor; i++)
    {
        MOTOR_REAL_INFO[i].unitMode = unitMode::SPEED_CONTROL_MODE;
        core->motor[i].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 3000, 3000, 15.0f);
//		core->motor[i].Motor_PID_RPM.PID_Parameter_Deinit(16.5f, 0.19f, 2.6f, 3000, 3000, 0.5f);
		
//        core->motor[i].Motor_PID_POS.PID_Parameter_Deinit(100.0f, 0, 1.0f, 7000, 7000, 0.05f);
    }
}

// 默认构造函数，初始化 3 个电机实例
Motor_Manager::Motor_Manager() : sizeof_Motor(3) {}

// 自定义构造函数，初始化指定数量的电机实例
Motor_Manager::Motor_Manager(uint_fast8_t size)
{
    sizeof_Motor = (size > MAX_MOTORS) ? MAX_MOTORS : size;
}

void Motor_Manager::Motor_Control(void)
{
    Motor_CAN_update(&latestCAN1_Message.RxHeader, latestCAN1_Message.RxData);
    for (int i = 0; i < sizeof_Motor; i++)
    {
        switch (MOTOR_REAL_INFO[i].unitMode)
        {
            case SPEED_CONTROL_MODE:
                motor[i].Motor_PID_RPM.PID_incremental_PID_calculation(MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);
                MOTOR_REAL_INFO[i].TARGET_CURRENT = motor[i].Motor_PID_RPM.Get_Output();
                break;
            default:
                break;
        }
    }
    Motor_CURRENT_CAN_send();
}

void Motor_Manager::Motor_CAN_update(CAN_RxHeaderTypeDef *msg, uint8_t *can1_RxData)
{
    switch (msg->StdId)
    {
        case MotorR_ID_1:
            MOTOR_REAL_INFO[0].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];
            MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];
            MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];
            break;
        case MotorR_ID_2:
            MOTOR_REAL_INFO[1].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];
            MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];
            MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];
            break;
        case MotorR_ID_3:
            MOTOR_REAL_INFO[2].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];
            MOTOR_REAL_INFO[2].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];
            MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];
            break;
        default:
            break;
    }
}

void Motor_Manager::Motor_CURRENT_CAN_send(void)
{
    CAN_TxHeaderTypeDef tx_message_1;
	static uint8_t send_buf1[8] = {0};
    uint32_t msg_box1;

    tx_message_1.IDE = CAN_ID_STD;
    tx_message_1.RTR = CAN_RTR_DATA;
	tx_message_1.TransmitGlobalTime = DISABLE;
    tx_message_1.DLC = 0x08;
    tx_message_1.StdId = 0x200;

   
		send_buf1[0] = (uint8_t)(MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
		send_buf1[1] = (uint8_t)(MOTOR_REAL_INFO[0].TARGET_CURRENT & 0xFFFF);
		send_buf1[2] = (uint8_t)(MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
        send_buf1[3] = (uint8_t)MOTOR_REAL_INFO[1].TARGET_CURRENT & 0xFFFF;
		send_buf1[4] = (uint8_t)(MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
        send_buf1[5] = (uint8_t)MOTOR_REAL_INFO[2].TARGET_CURRENT & 0xFFFF;
		send_buf1[6] = (uint8_t)(MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
        send_buf1[7] = (uint8_t)MOTOR_REAL_INFO[3].TARGET_CURRENT & 0xFFFF;


    if (HAL_CAN_AddTxMessage(&hcan1, &tx_message_1, send_buf1, &msg_box1) != HAL_OK)
    {
        // 处理发送失败
    }
}

void Motor_Manager::Task_Function(void)
{
    Motor_Control();
}
