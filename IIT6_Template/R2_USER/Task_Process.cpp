#include "Task_Process.h"

void My_Task_Creator(void){
    //创建实例
    Motor_Manager motor_core;
    xbox xbox_core;
    Calculation calculation_core;
    Peripheral_Control peripheral_core;
    Task_Manager task_core;



    //注册任务（）
    task_core.registerTask(0, &xbox_core);
    task_core.registerTask(1, &calculation_core);
    task_core.registerTask(2, &motor_core);
    task_core.registerTask(2, &peripheral_core);

    //启动freertos内核
    osKernelStart();
}