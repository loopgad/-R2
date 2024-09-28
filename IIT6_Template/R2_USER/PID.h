#ifndef __PID_H
#define __PID_H


#include "Global_Namespace.h"


#define ABS(x)      ((x)>0? (x):(-(x)))


//使用时可直接构建class数组，用电机参数数组进行循环赋值
class PID_Class{
  private:
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
