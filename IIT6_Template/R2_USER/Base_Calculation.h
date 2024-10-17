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


using namespace Xbox_Namespace;
using namespace Action_Namespace;
#define  push 1 
#define  back -1
#define	 hold  0
#define RADIUS 						0.1730520 // // 轮子（中心）到底盘圆心的距离（近似
#define Radius_Wheel				0.0719

	class Calculation:public Task_Thread {
		private:
			void Kinematic_Analysis_Inverse(void);                
			void Axis_analyse_for_WORLDtoROBOT(void);
			void World_Control(void);
			void Robot_Control(void);
		public:
			void Task_Function();
   
	};


//#endif

#endif 