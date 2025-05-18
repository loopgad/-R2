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
<<<<<<< HEAD
        .priority = (osPriority_t) osPriorityRealtime,
=======
        .priority = (osPriority_t) osPriorityNormal2,
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
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
<<<<<<< HEAD
        .priority = (osPriority_t) osPriorityNormal3,
=======
        .priority = (osPriority_t) osPriorityRealtime,
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
    };
}

// 函数声明
void Xbox_Task(void *argument);
void Calculation_Task(void *argument);
void Peripheral_Control_Task(void *argument);
void Motor_Control_Task(void *argument);
void My_Task_Creator(void);
<<<<<<< HEAD
=======

//调试用全局变量
extern float data_kp;
extern float data_ki;
extern float data_kd;
extern float data_output;
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
