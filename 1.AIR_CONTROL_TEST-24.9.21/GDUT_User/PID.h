#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

#define ABS(x)      ((x)>0? (x):(-(x)))
typedef struct PID 
{
  float  Proportion;         // ��������  
  float  Integral;           // ���ֳ���  
  float  Derivative;         // ΢�ֳ���  
	float  PrevError;          // ǰǰ�����  
  float  LastError;          // ǰ�����  
	float  Error;              // ��ǰ���  
	float  DError;             // ���仯��  
  float  SumError;           // ����ۻ���  
	float  Integralmax;        // ��������  
	float  output;             // ���ֵ  
	float  outputmax;          // �������  
	float  errormax;           // �������  
	uint8_t first_flag;        // �״����б�־  
	float  deadzone;           // ����  
}PID;


void PID_parameter_init(PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone);
float PID_abs_limit(float a, float ABS_MAX);
void PID_incremental_PID_calculation(PID *pp,  float CurrentPoint, float NextPoint);
void PID_incremental_PID_calculation_by_error(PID *pp, float error);
void PID_position_PID_calculation(PID *pp, float CurrentPoint, float NextPoint);
void PID_position_PID_calculation_by_error(PID *pp, float error);
void PID_reset_PID(PID *pp);


#endif