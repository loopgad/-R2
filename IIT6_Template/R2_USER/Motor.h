#ifndef __MOTOR_H
#define __MOTOR_H

//宏定义
#define L 0.1730f;
#define number 1.7320f		//sqrt(3)
#define PI 3.14159265358979323846


#include "Global_Namespace.h"
#include "PID.h"
#include "Task_Manager.h"




class Motor:public Task_Thread
{
  private:
    PID_Class PID[8][3];//8个电机，每个电机3个pid，分别为位置pid，速度pid，角度pid

  public:
    void M3508_Motor_Init(void);

};





#endif