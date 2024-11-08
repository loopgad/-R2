#include "Task_Process.h"


using namespace Task_Namespace;

void My_Task_Creator(void) {
    // 创建任务
    xbox_core_handle = osThreadNew(Xbox_Task, &xbox_core, &xbox_core_attributes);
    calculation_core_handle = osThreadNew(Calculation_Task, &calculation_core, &calculation_core_attributes);
    peripheral_core_handle = osThreadNew(Peripheral_Control_Task, &peripheral_core, &peripheral_core_attributes);
    motor_core_handle = osThreadNew(Motor_Control_Task, &motor_core, &motor_core_attributes);
}


// 各任务函数的实现
void Xbox_Task(void *argument) {
    xbox* core = static_cast<xbox*>(argument);  // 将参数转换为 xbox 指针
    for(;;) {
        core->Task_Function();  // 调用成员函数      
        osDelay(1);  // 控制输出频率
    }
}

void Calculation_Task(void *argument) {
    Calculation* core = static_cast<Calculation*>(argument);  // 转换为 Calculation 指针
    for(;;) {
        core->Task_Function();
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
        core->Task_Function();
        osDelay(1);
    }
}
