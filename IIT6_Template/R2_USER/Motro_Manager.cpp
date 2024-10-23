#include "Motor_Manager.h"


//Motor m3508[3];  创建电机类实例

/**
  * @brief  电机组重初始化
	* @param  None
	* @retval None
  * @attention  在main调用一下初始化
  *电机初始化（包含PID，电机类型）
  */
void  Motor_Init(Motor Motor[], uint_fast8_t sizeof_Motor)
{
	//PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone
	
	//速度环
    for(int i = 0; i < sizeof_Motor; i++){
        Motor[i].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
	    //m3508[1].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
	    //m3508[2].Motor_PID_RPM.PID_Parameter_Deinit(12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);

	//位置环pid
	    Motor[i].Motor_PID_POS.PID_Parameter_Deinit(100, 0, 1, 7000, 7000, 0.05);
	    //m3508[1].Motor_PID_POS.PID_Parameter_Deinit(100, 0, 1, 7000, 7000, 0.05);
	    //m3508[2].Motor_PID_POS.PID_Parameter_Deinit(100, 0, 1, 7000, 7000, 0.05);
}
	




void Motor_Manager::Task_Function(){
	
}