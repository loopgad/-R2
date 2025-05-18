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

#ifndef __PID_H
#define __PID_H


#include "Global_Namespace.h"


#define ABS(x)      ((x)>0? (x):(-(x)))


//使用时可直接构建class数组，用电机参数数组进行循环赋值
class PID_Class{
  private:
    typedef struct PID 
	{
		float  Proportion;         // 比例常数
		float  Integral;           // 积分常数
		float  Derivative;         // 微分常数
		float  Penu_Error;          // 倒数第二次误差  
		float  Prev_Error;          // 上一次误差  
		float  Error;              // 当前误差  
		float  Delta_Error;             //误差变化率  
		float  Sum_Error;           // 误差积累和  
		float  Integral_max;        // 积分上限  
		float  output;             //  输出值 
		float  output_max;          //  输出上限
		float  error_max;           //  误差上限 
		bool first_flag;        //  首次运行标志位
		float  deadzone;           //  死区 
	}PID;
	
	PID pp = {};
	
  public:
    PID_Class();
	void PID_Parameter_Deinit(float Kp, float Ki, float Kd, float output_max, float Integral_max, float deadzone);
	float PID_abs_limit(float a, float ABS_MAX);
	void PID_incremental_PID_calculation(float CurrentPoint, float NextPoint);
	void PID_incremental_PID_calculation_by_error(float error);
	void PID_position_PID_calculation(float CurrentPoint, float NextPoint);
	void PID_position_PID_calculation_by_error(float error);
	void PID_reset();
	float Get_Output(void);

};




#endif
