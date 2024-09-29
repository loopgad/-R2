#ifndef BASE_CALCULATION_H
#define BASE_CALCULATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "math.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "Global_Namespace.h"
using namespace Remote_Namespace;
using namespace Action_Namespace;
#define  push 1 
#define  back -1
#define	 hold  0
#define RADIUS 						0.1730520 // // 轮子（中心）到底盘圆心的距离（近似
#define Radius_Wheel				0.0719

	class Calculation {
		public:
			void Kinematic_Analysis_Inverse(void);                
			void Axis_analyse_for_WORLDtoROBOT(void);
			void World_Control(void);
			void Robot_Control(void);
   
	};


#endif

#endif 