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

#include "PID.h"


//默认用RPM的pid参数构造类
PID_Class::PID_Class(){
	pp.Integral_max = 10000;
	pp.output_max = 10000;
	pp.Proportion = 12.0f;
	pp.Integral   = 0.4f;
	pp.Derivative = 0.1f;
    pp.Delta_Error = pp.Error = pp.Sum_Error = pp.output = pp.Prev_Error = pp.Penu_Error = pp.error_max = 0.0f;
	pp.first_flag = true;
	pp.deadzone = 0.5f;
}

//重初始化参数
void PID_Class::PID_Parameter_Deinit(float Kp, float Ki, float Kd, float output_max, float Integral_max, float deadzone){
    pp.Integral_max = Integral_max;
	pp.output_max = output_max;
	pp.Proportion = Kp;
	pp.Integral   = Ki;
	pp.Derivative = Kd;
    pp.Delta_Error = pp.Error = pp.Sum_Error = pp.output = pp.Prev_Error = pp.Penu_Error = pp.error_max = 0.0f;
	pp.first_flag = true;
	pp.deadzone = deadzone;
}

//PID限幅
float PID_Class::PID_abs_limit(float a, float ABS_MAX){
    if(a > ABS_MAX)
        a = ABS_MAX;
		
    if(a < -ABS_MAX)
        a = -ABS_MAX;
		return a;
}

//增量式PID
void PID_Class::PID_incremental_PID_calculation( float CurrentPoint, float NextPoint){
	pp.Error =  NextPoint - CurrentPoint;                               
	pp.Delta_Error = pp.Error - pp.Prev_Error;
	
	pp.output +=  pp.Proportion * (pp.Delta_Error)+   \
			       PID_abs_limit(pp.Integral * pp.Error, pp.Integral_max ) +   \
				   pp.Derivative * ( pp.Error +  pp.Penu_Error - 2*pp.Prev_Error);  

    //限幅
	if(pp.output > pp.output_max )
    {
        pp.output = pp.output_max;
    }
	if(pp.output < - pp.output_max )
    {
        pp.output = -pp.output_max;
    }  

    //更新误差	
	pp.Penu_Error = pp.Prev_Error;  
	pp.Prev_Error = pp.Error;
	
	if(ABS(pp.Error) < pp.deadzone)
	{
		pp.output = 0;
	}
}

//直接传入误差的增量式PID
void PID_Class::PID_incremental_PID_calculation_by_error(float error){
    pp.Error = error;                               
	pp.Delta_Error = pp.Error - pp.Prev_Error;
	
	pp.output +=  pp.Proportion * (pp.Delta_Error)+   \
				   pp.Integral * pp.Error +  \
				   pp.Derivative * ( pp.Error +  pp.Penu_Error - 2*pp.Prev_Error);  

	if(pp.output > pp.output_max )  
    {
        pp.output = pp.output_max;
    }
	if(pp.output < - pp.output_max )  
	{
        pp.output = -pp.output_max;
    }	
        
	pp.Penu_Error = pp.Prev_Error;  
	pp.Prev_Error = pp.Error;
	
	if(ABS(pp.Error) < pp.deadzone)
	{
		pp.output = 0;
	}   
}

//位置式PID
void PID_Class::PID_position_PID_calculation(float CurrentPoint, float NextPoint){
    if(pp.first_flag)
	{
		pp.Prev_Error = NextPoint - CurrentPoint;
		pp.Penu_Error = NextPoint - CurrentPoint;
		pp.first_flag = false;
	}
	
	pp.Error =  NextPoint -  CurrentPoint;          
	pp.Sum_Error += pp.Error;                      
	pp.Delta_Error = pp.Error - pp.Prev_Error;
	
	pp.output =  pp.Proportion * pp.Error +   \
				  PID_abs_limit(pp.Integral * pp.Sum_Error, pp.Integral_max ) +   \
				  pp.Derivative * pp.Delta_Error ;  

	if(pp.output > pp.output_max )  pp.output = pp.output_max;
	if(pp.output < - pp.output_max )  pp.output = -pp.output_max; 
	pp.Prev_Error = pp.Error;
	
	if(ABS(pp.Error) < pp.deadzone)
	{
		pp.output = 0;
	}
}

//直接传入误差的位置式PID
void PID_Class::PID_position_PID_calculation_by_error(float error){
    if(pp.first_flag)
	{
		pp.Prev_Error = error;
		pp.Penu_Error = error;
		pp.first_flag = false;
	}	
	
	pp.Error =  error;          
	pp.Sum_Error += pp.Error;                      
	pp.Delta_Error = pp.Error - pp.Prev_Error;
	
	pp.output =  pp.Proportion * pp.Error +   \
				  PID_abs_limit(pp.Integral * pp.Sum_Error, pp.Integral_max ) +   \
				  pp.Derivative * pp.Delta_Error ;  

	if(pp.output > pp.output_max )  pp.output = pp.output_max;
	if(pp.output < - pp.output_max )  pp.output = -pp.output_max; 
	pp.Prev_Error = pp.Error;
	
	if(ABS(pp.Error) < pp.deadzone)
	{
		pp.output = 0;
	}
}

//重置PID启动标志位
void PID_Class::PID_reset(){
    pp.Delta_Error = pp.Error = pp.Sum_Error = pp.output = pp.Prev_Error = pp.Penu_Error = pp.error_max = 0.0f; 
	pp.first_flag = true;
}

float PID_Class::Get_Output(void){
	return pp.output;
}

