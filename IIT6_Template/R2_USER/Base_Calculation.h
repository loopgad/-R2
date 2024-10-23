#ifndef BASE_CALCULATION_H
#define BASE_CALCULATION_H

//#ifdef __cplusplus
//extern "C"
//{
//#endif

#include "Global_Namespace.h"
#include "Task_Manager.h"

//#ifdef __cplusplus
//}
//#endif

//#ifdef __cplusplus
using namespace ROBOT_Namespace;
using namespace Motor_Namespace;
using namespace ROS_Namespace;
using namespace Xbox_Namespace;
using namespace Action_Namespace;

#define  push 1 
#define  back -1
#define	 hold  0
#define RADIUS 						0.1730520 	 // 轮子（中心）到底盘圆心的距离（近似
#define Radius_Wheel				0.0719
#define PI   						3.141592654
enum direction{
	y=0,
	x=1,
	w=2
};
	class Calculation:public Task_Thread {
		private:
			float R;//用于记录与敌方车的距离
			float Vx_temp;//temporary,临时储存和速度
			float Vy_temp;
			float Alpha;//计算记录对对方车的角度，弧度制
		
			/*丐版PD，锁头用*/
			float KEEP_YAW;
			float NOW_YAW;//不用real是为了和一堆real_yaw区分开来
			bool Readed_Flag=0;
			float Err_Yaw;
			float Last_Yaw=0;
			float Output;
		
			inline void Kinematic_Analysis_Inverse(void);                
			inline void Axis_analyse_for_WORLDtoROBOT(void);
			void World_Control(void);
			void Robot_Control(void);
			void Semi_auto_Control(void);
			void Move_State(void);
			void YawAdjust(float Target_Yaw);
		public:
			void Task_Function();
   
	};


//#endif

#endif 