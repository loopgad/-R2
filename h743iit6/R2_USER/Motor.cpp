/*
Copyright (c) 2024 loopgad 9th_R2_Member

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "Motor.h"



void Motor_Init(Motor_Manager* core,uint_fast8_t sizeof_Motor) //重定义可用
{

    for (int i = 0; i < sizeof_Motor; i++)
    {
        Motor_Namespace::MOTOR_REAL_INFO[i].unitMode =  Motor_Namespace::unitMode::SPEED_CONTROL_MODE;
        core->motor[i].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 3000, 3000, 15.0f);
//		core->motor[i].Motor_PID_RPM.PID_Parameter_Deinit(16.5f, 0.19f, 2.6f, 3000, 3000, 0.5f);
		
//        core->motor[i].Motor_PID_POS.PID_Parameter_Deinit(100.0f, 0, 1.0f, 7000, 7000, 0.05f);
    }
}

// 默认构造函数，初始化 3 个电机实例
Motor_Manager::Motor_Manager() : sizeof_Motor(3) {
}

// 自定义构造函数，初始化指定数量的电机实例
Motor_Manager::Motor_Manager(uint_fast8_t size)
{
    sizeof_Motor = (size > MAX_MOTORS) ? MAX_MOTORS : size;
}

void Motor_Manager::Motor_Control(void)
{
    //Motor_CAN_update(&latestCAN1_Message.RxHeader, latestCAN1_Message.RxData); 需要重写
    for (int i = 0; i < sizeof_Motor; i++)
    {
        switch (Motor_Namespace::MOTOR_REAL_INFO[i].unitMode)
        {
            case  Motor_Namespace::SPEED_CONTROL_MODE:
                motor[i].Motor_PID_RPM.PID_incremental_PID_calculation(Motor_Namespace::MOTOR_REAL_INFO[i].RPM, Motor_Namespace::MOTOR_REAL_INFO[i].TARGET_RPM);
                Motor_Namespace::MOTOR_REAL_INFO[i].TARGET_CURRENT = motor[i].Motor_PID_RPM.Get_Output();
                break;
            default:
                break;
        }
    }
    //Motor_CURRENT_CAN_send(); 需要重写
}

void Motor_Manager::Motor_CAN_update(FDCAN_RxHeaderTypeDef *msg, uint8_t *can1_RxData)
{
    switch (msg->IdType)
    {
        case MotorR_ID_1:
            Motor_Namespace::MOTOR_REAL_INFO[0].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];
            Motor_Namespace::MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];
            Motor_Namespace::MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];
            break;
        case MotorR_ID_2:
            Motor_Namespace::MOTOR_REAL_INFO[1].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];
            Motor_Namespace::MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];
            Motor_Namespace::MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];
            break;
        case MotorR_ID_3:
            Motor_Namespace::MOTOR_REAL_INFO[2].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];
            Motor_Namespace::MOTOR_REAL_INFO[2].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];
            Motor_Namespace::MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];
            break;
        default:
            break;
    }
}

/***********************need to override***************************/
//void Motor_Manager::Motor_CURRENT_CAN_send(void)
//{
//    FDCAN_TxHeaderTypeDef tx_message_1;
//    static uint8_t send_buf1[8] = {0};
//    uint32_t msg_box1;

//    // 设置FDCAN为传统CAN模式
//    tx_message_1.IdType = CAN_ID_STD;
//    tx_message_1.RTR = CAN_RTR_DATA;
//    tx_message_1.TransmitGlobalTime = DISABLE;
//    tx_message_1.Stamp = 0; // 这里应该是Stamp，而不是点号
//    tx_message_1.Identifier = 0x200;

//    // 填充发送缓冲区
//    send_buf1[0] = (uint8_t)(Motor_Namespace::MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
//    send_buf1[1] = (uint8_t)(Motor_Namespace::MOTOR_REAL_INFO[0].TARGET_CURRENT);
//    send_buf1[2] = (uint8_t)(Motor_Namespace::MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
//    send_buf1[3] = (uint8_t)Motor_Namespace::MOTOR_REAL_INFO[1].TARGET_CURRENT;
//    send_buf1[4] = (uint8_t)(Motor_Namespace::MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
//    send_buf1[5] = (uint8_t)Motor_Namespace::MOTOR_REAL_INFO[2].TARGET_CURRENT;
//    send_buf1[6] = (uint8_t)(Motor_Namespace::MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
//    send_buf1[7] = (uint8_t)Motor_Namespace::MOTOR_REAL_INFO[3].TARGET_CURRENT;

//    // 发送CAN消息
//    if (HAL_FDCAN_AddTxMessage(&hcan1, &tx_message_1, send_buf1, &msg_box1) != HAL_OK)
//    {
//        // 处理发送失败
//    }
//}


//void Motor_Manager::Task_Function(void)
//{
// Motor_CAN_update(&latestCAN1_Message.RxHeader, latestCAN1_Message.RxData);
//// 	Motor_CURRENT_CAN_send();
//    Motor_Control();
//    
//}
