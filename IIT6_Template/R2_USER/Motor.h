#ifndef __MOTOR_H
#define __MOTOR_H

//宏定义
#define L 0.1730f;
#define number 1.7320f		//sqrt(3)
#define PI 3.14159265358979323846
#include "Global_Namespace.h"
#include "PID.h"
#include "Task_Manager.h"

enum UnitMode {
    POSITION_CONTROL_MODE,
    SPEED_CONTROL_MODE,
    CURRENT_MODE,
    MOTO_OFF
};

using namespace Motor_Namespace;
class Motor
{
   private:
    PID_Class MOTOR_PID[3];   // 每个电机3个pid，分别为位置pid，速度pid，角度pid
    PID_Class &PID_POS;       // 非静态引用，指向 MOTOR_PID[0]
    PID_Class &PID_PRM;       // 非静态引用，指向 MOTOR_PID[1]
    PID_Class &PID_YAW;       // 非静态引用，指向 MOTOR_PID[2]
    int ID;
  public:
    Motor(int ID_NUM);//創建電機對象時用ID初始化
    inline void Motor_PID_Init(void);
    void MotorCtrl(void);
    
};

void Send_Motor_Currents(void);//在每一個電機類的輸出計算完后再調用



#endif