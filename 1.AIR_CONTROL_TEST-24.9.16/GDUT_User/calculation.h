/**
  ******************************************************************************
  * @file    
  * @author  LLY
  * @version 1.0
  * @date    2023/3/16
  * @brief   移植东北大学rm3508_can_opensource-master 
  ******************************************************************************
  * @attention
  *
	*
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __calculation_h
#define __calculation_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"
#include "moto.h"
#include "HareWare.h"


#define  push 1 
#define  back -1
#define	 hold  0

#define RADIUS 						0.1730520 // 轮子（中心）到底盘圆心的距离（近似
#define Radius_Wheel				0.0719
/* Exported types ------------------------------------------------------------*/
;

/* functions ------------------------------------------------------- */
void Kinematic_Analysis_Inverse(void);
void Axis_analyse_for_WORLDtoROBOT(void);
void World_Control(void);
void Robot_Control(void);
void AUTO_Control(void);
void Contor_FSM(void);

#endif


