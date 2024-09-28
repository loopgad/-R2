#include "PID.h"


//用于传入在对象创建时初始化参数，不传入结构体创建的对象需要主动调用Deinit函数初始化参数
void PID_Class::PID_Parameter_Init(PID *pp_parameter){
    pp.Integral_max = pp_parameter->Integral_max;
	pp.output_max = pp_parameter->output_max;
	pp.Proportion = pp_parameter->Proportion;
	pp.Integral   = pp_parameter->Integral;
	pp.Derivative = pp_parameter->Derivative;
    pp.Delta_Error = pp.Error = pp.Sum_Error = pp.output = pp.Prev_Error = pp.Penu_Error = pp.error_max = 0.0f;
	pp.first_flag = true;
	pp.deadzone = pp_parameter->deadzone;
}

//对PID参数进行初始化
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

//对变量进行限幅
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

    //更新变量	
	pp.Penu_Error = pp.Prev_Error;  
	pp.Prev_Error = pp.Error;
	
	if(ABS(pp.Error) < pp.deadzone)
	{
		pp.output = 0;
	}
}

//增量式PID(直接传入误差)
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

//位置式PID(直接传入误差)
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

//重置PID运行标志位和参数
void PID_Class::PID_reset(){
    pp.Delta_Error = pp.Error = pp.Sum_Error = pp.output = pp.Prev_Error = pp.Penu_Error = pp.error_max = 0.0f; 
	pp.first_flag = true;
}

