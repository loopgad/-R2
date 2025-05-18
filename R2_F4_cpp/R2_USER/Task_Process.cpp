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
#include "Task_Process.h"


using namespace Task_Namespace;

<<<<<<< HEAD
void My_Task_Creator(void) {
    // 创建任务
    xbox_core_handle = osThreadNew(Xbox_Task, &xbox_core, &xbox_core_attributes);
    calculation_core_handle = osThreadNew(Calculation_Task, &calculation_core, &calculation_core_attributes);
    peripheral_core_handle = osThreadNew(Peripheral_Control_Task, &peripheral_core, &peripheral_core_attributes);
    motor_core_handle = osThreadNew(Motor_Control_Task, &motor_core, &motor_core_attributes);
=======
float delta = 0;
uint32_t current_time;
uint32_t next_current_time;
Serialport_Drive VOFA;
void My_Task_Creator(void) {
    // 创建任务
    //xbox_core_handle = osThreadNew(Xbox_Task, &xbox_core, &xbox_core_attributes);
    //calculation_core_handle = osThreadNew(Calculation_Task, &calculation_core, &calculation_core_attributes);
    //peripheral_core_handle = osThreadNew(Peripheral_Control_Task, &peripheral_core, &peripheral_core_attributes);
    motor_core_handle = osThreadNew(Motor_Control_Task, &motor_core, &motor_core_attributes);

>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
}


// 各任务函数的实现
void Xbox_Task(void *argument) {
    xbox* core = static_cast<xbox*>(argument);  // 将参数转换为 xbox 指针
    for(;;) {
<<<<<<< HEAD
        core->Task_Function();  // 调用成员函数      
        osDelay(1);  // 控制输出频率
=======
    
        core->Task_Function();  // 调用成员函数     
        osDelay(5);  // 控制输出频率
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
    }
}

void Calculation_Task(void *argument) {
    Calculation* core = static_cast<Calculation*>(argument);  // 转换为 Calculation 指针
    for(;;) {
<<<<<<< HEAD
        core->Task_Function();
=======

        core->Task_Function();    
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
        osDelay(2);
    }
}

void Peripheral_Control_Task(void *argument) {
    Peripheral_Control* core = static_cast<Peripheral_Control*>(argument);  // 转换为 Peripheral_Control 指针
    for(;;) {
        core->Task_Function();
        osDelay(1);
    }
}

void Motor_Control_Task(void *argument) {
    Motor_Manager* core = static_cast<Motor_Manager*>(argument);  // 转换为 Motor_Manager 指针
	Motor_Init(core, core->sizeof_Motor);
    for(;;) {
<<<<<<< HEAD
        core->Task_Function();
        osDelay(1);
=======

        core->Task_Function();
        //以下代码为调试用
    for (int i = 0; i < core->sizeof_Motor; i++)
    {
        Motor_Namespace::MOTOR_REAL_INFO[i].unitMode =  Motor_Namespace::unitMode::SPEED_CONTROL_MODE;
        core->motor[i].Motor_PID_RPM.PID_Parameter_Deinit(data_kp, data_ki, data_kd, 6000, 6000, 5.0f);

//		core->motor[i].Motor_PID_RPM.PID_Parameter_Deinit(16.5f, 0.19f, 2.6f, 3000, 3000, 0.5f);
		
//      core->motor[i].Motor_PID_POS.PID_Parameter_Deinit(100.0f, 0, 1.0f, 7000, 7000, 0.05f);
    }
    Motor_Namespace::MOTOR_REAL_INFO[0].TARGET_RPM = data_output;

    osDelay(5);
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
    }
}
