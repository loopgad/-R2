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
	
	PID pp;

  public:
    void PID_Parameter_Init(PID *pp_parameter);
	void PID_Parameter_Deinit(float Kp, float Ki, float Kd, float output_max, float Integral_max, float deadzone);
	float PID_abs_limit(float a, float ABS_MAX);
	void PID_incremental_PID_calculation(float CurrentPoint, float NextPoint);
	void PID_incremental_PID_calculation_by_error(float error);
	void PID_position_PID_calculation(float CurrentPoint, float NextPoint);
	void PID_position_PID_calculation_by_error(float error);
	void PID_reset();

};




#endif
