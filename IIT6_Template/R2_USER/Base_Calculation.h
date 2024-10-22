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
			inline void Kinematic_Analysis_Inverse(void);                
			inline void Axis_analyse_for_WORLDtoROBOT(void);
			void World_Control(void);
			void Robot_Control(void);
			void Semi_auto_Control(void);
			void Move_State(void);
		public:
			void Task_Function();
   
	};


//#endif

#endif 