<<<<<<< HEAD
=======
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

>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
#ifndef Motor_H
#define Motor_H


#include "Global_Namespace.h"
#include "PID.h"
#include "can.h"

// 最大电机数量
#define MAX_MOTORS 8 

// M3508电机编号
#define MotorR_ID_1      0x201
#define MotorR_ID_2      0x202
#define MotorR_ID_3      0x203



extern CAN_HandleTypeDef hcan1;

// 电机类
class Motor
{
public:
    PID_Class Motor_PID_RPM;  // 速度 PID 信息
    PID_Class Motor_PID_POS;  // 位置 PID 信息
};

// 电机管理员类
class Motor_Manager
{
private:
    
    void Motor_CAN_update(CAN_RxHeaderTypeDef *msg, uint8_t can1_RxData[8]);
    void Motor_CURRENT_CAN_send(void);

public:
	Motor motor[MAX_MOTORS];    // 固定大小的电机数组
    uint_fast8_t sizeof_Motor;  // 电机实例数量
    Motor_Manager();
    Motor_Manager(uint_fast8_t size);
    void Motor_Control(void);
    void Task_Function(void);
    friend void Motor_Init(uint_fast8_t sizeof_Motor);
};

// 电机初始化函数
void Motor_Init(Motor_Manager* core,uint_fast8_t sizeof_Motor);

extern CAN_RX_Message_t latestCAN1_Message; // 在 can.h 中定义

#endif
