#pragma once

#include "Global_Namespace.h"
#include "Base_Calculation.h"
#include "Motor.h"
#include "Xbox_Control.h"
#include "Peripheral_Control_Unit.h"
#include "Callback_Function.h"
#include "FreeRTOS.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

#ifdef __cplusplus
}
#endif


namespace Task_Namespace {
    // 定义全局实例变量，使用 inline 特性
    inline xbox xbox_core;
    inline Calculation calculation_core;
    inline Peripheral_Control peripheral_core;
    inline Motor_Manager motor_core;

    // 定义任务句柄
    inline osThreadId_t xbox_core_handle;
    inline osThreadId_t calculation_core_handle;
    inline osThreadId_t peripheral_core_handle;
    inline osThreadId_t motor_core_handle;

    // 定义任务属性
    inline const osThreadAttr_t xbox_core_attributes = {
        .name = "xbox_core",
        .stack_size = 256 * 3,
        .priority = (osPriority_t) osPriorityRealtime,
    };
	
    inline const osThreadAttr_t calculation_core_attributes = {
        .name = "calculation_core",
        .stack_size = 256 * 3,
        .priority = (osPriority_t) osPriorityNormal2,
    };
	
    inline const osThreadAttr_t peripheral_core_attributes = {
        .name = "peripheral_core",
        .stack_size = 256 * 3,
        .priority = (osPriority_t) osPriorityNormal3,
    };
	
    inline const osThreadAttr_t motor_core_attributes = {
        .name = "motor_core",
        .stack_size = 256 * 3,
        .priority = (osPriority_t) osPriorityNormal3,
    };
}

// 函数声明
void Xbox_Task(void *argument);
void Calculation_Task(void *argument);
void Peripheral_Control_Task(void *argument);
void Motor_Control_Task(void *argument);
void My_Task_Creator(void);
