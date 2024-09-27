#include "Task_Manager.h"

Task_Info Task_Manager::tasks[MAX_TASKS]; //定义了一个task结构体列表用于储存任务信息

Task_Manager::Task_Manager()
{
    for (int i = 0; i < MAX_TASKS; i++)
    {
        tasks[i].name[0] = 't';
        tasks[i].name[1] = 'a';
        tasks[i].name[2] = 's';
        tasks[i].name[3] = 'k';
        tasks[i].name[4] = '_';
        tasks[i].name[5] = '0' + i;
        tasks[i].name[6] = '\0';
        // 生成一波任务名字，便于后续debug
    }
    // 默认的任务参数设置，如有不满可以走客制化
    tasks[0].Priority = 4;
    tasks[0].delay_ms = 1;
    tasks[1].Priority = 5;
    tasks[1].delay_ms = 1;
    tasks[2].Priority = 5;
    tasks[2].delay_ms = 1;
    tasks[3].Priority = 5;
    tasks[3].delay_ms = 1;
    tasks[4].Priority = 3;
    tasks[4].delay_ms = 3;
    tasks[5].Priority = 2;
    tasks[5].delay_ms = 3;
    tasks[6].Priority = 2;
    tasks[6].delay_ms = 5;
    tasks[7].Priority = 1;
    tasks[7].delay_ms = 5;
    tasks[8].Priority = 1;
    tasks[8].delay_ms = 10;
    tasks[9].Priority = 0;
    tasks[9].delay_ms = 10;
}



void Task_Manager::TaskFunction_Handle(void *parameters)
{

    int run_taskID = (int)(uintptr_t)parameters;
    // TaskInfo &taskInfo = tasks[taskID];

    while (true)
    {
        for (int i = 0; i < tasks[run_taskID].instanceCount; ++i)
        { // 执行所有实例
            if (tasks[run_taskID].instances[i])
            {
                tasks[run_taskID].instances[i]->Task_Function();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(tasks[run_taskID].delay_ms)); // 延时
    }
}



void Task_Manager::customize(int taskID, uint8_t priority, uint32_t delay_ms, uint32_t stack_size)
{
    if (tasks[taskID].handle != nullptr)
    {
        return; // 已创建的任务无法修改参数
    }
    tasks[taskID].Priority = priority;
    tasks[taskID].delay_ms = delay_ms;
    tasks[taskID].stack_size = stack_size;
}



void Task_Manager::registerTask(int taskID, Task_Thread *instance)//Task_Thread *instance为多态性，允许将子类对象作为父类对象处理
{
    if (taskID < 0 || taskID >= MAX_TASKS || tasks[taskID].instanceCount >= MAX_CLASSES_PER_TASK)
    {
        return;
    }

    // 注册任务信息，方便debug
    tasks[taskID].instances[tasks[taskID].instanceCount] = instance;
    tasks[taskID].instanceCount++;

    // 如果任务还未创建，则创建任务
    if (tasks[taskID].handle == nullptr)
    {
        xTaskCreate(TaskFunction_Handle, tasks[taskID].name, tasks[taskID].stack_size, (void *)taskID, tasks[taskID].Priority, &tasks[taskID].handle);
    }
}


