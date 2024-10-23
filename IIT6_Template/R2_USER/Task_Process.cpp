#include "Task_Process.h"

extern "C" void Task_Creator(void){
    //创建实例
    Motor_Manager motor_core;
    xbox xbox_core;
    Calculation base_calculation;
    Task_Manager task_core;



    //注册任务（）
    task_core.registerTask(0, &xbox_core);
    task_core.registerTask(1, &motor_core);
    task_core.registerTask(2, &motor_core);

    //启动freertos内核
    osKernelStart();
}