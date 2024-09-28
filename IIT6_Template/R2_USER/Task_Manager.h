
#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <FreeRTOS.h>
#include <cmsis_os.h>
#include <cstdint>
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// 最大任务数和每个任务允许的类实例数
#define MAX_TASKS 10
#define MAX_CLASSES_PER_TASK 20

//所有需要在FREERTOS中直接执行循环任务的类都需要继承Task_Thread用于创建线程
    class Task_Thread
    {
        public:
        virtual void Task_Function() = 0; // 纯虚函数,需要在派生类重写，内部为需要循环执行的内容
        virtual ~Task_Thread() {}; 
        char name[20] = {'0'};           // 实例的名字(最大为20字节)，便于debug
    };


// 任务信息结构体
struct Task_Info
{
    TaskHandle_t handle = nullptr;
    char name[7] = {0}; //任务进程数信息
    uint8_t Priority = 0; // 在1到5之间
    uint32_t delay_ms = 1;
    Task_Thread *instances[MAX_CLASSES_PER_TASK] = {nullptr}; // 使用接口指针
    int instanceCount = 0;
    uint32_t stack_size = 1024;
};
	
	
// 任务管理者类
class Task_Manager
{

private:
    static Task_Info tasks[MAX_TASKS];//根据任务信息生成的任务列表
    static void TaskFunction_Handle(void *parameters); // 任务执行函数

public:
    Task_Manager();
    void registerTask(int taskID, Task_Thread *instance); // 为任务分配ID并创建任务线程
                                                             // 存储任务信息

    void customize(int taskID, uint8_t priority, uint32_t delay_ms, uint32_t stack_size = 2048); // 允许用户用指定的配置创建任务,但如果任务已经创建则无效

};



#endif

#endif // TASK_MANAGER_H
