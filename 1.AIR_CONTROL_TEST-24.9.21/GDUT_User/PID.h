#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

#define ABS(x)      ((x)>0? (x):(-(x)))
typedef struct PID 
{
  float  Proportion;         // 比例常数  
  float  Integral;           // 积分常数  
  float  Derivative;         // 微分常数  
	float  PrevError;          // 前前次误差  
  float  LastError;          // 前次误差  
	float  Error;              // 当前误差  
	float  DError;             // 误差变化率  
  float  SumError;           // 误差累积和  
	float  Integralmax;        // 积分上限  
	float  output;             // 输出值  
	float  outputmax;          // 输出上限  
	float  errormax;           // 误差上限  
	uint8_t first_flag;        // 首次运行标志  
	float  deadzone;           // 死区  
}PID;


void PID_parameter_init(PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone);
float PID_abs_limit(float a, float ABS_MAX);
void PID_incremental_PID_calculation(PID *pp,  float CurrentPoint, float NextPoint);
void PID_incremental_PID_calculation_by_error(PID *pp, float error);
void PID_position_PID_calculation(PID *pp, float CurrentPoint, float NextPoint);
void PID_position_PID_calculation_by_error(PID *pp, float error);
void PID_reset_PID(PID *pp);


#endif
