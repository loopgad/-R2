/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "gpio.h"
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

//定义位于main.c
extern uint32_t time_count; //计数变量

/* USER CODE BEGIN Private defines */


/**************************用于修改分度值**************************/
#define ms_division //ms
//#define tenth_ms_divsion //100us
//#define fiftieth_ms_division //20us


#if defined(ms_division)

	#define time_division_factor 1000

#elif defined(tenth_ms_divsion)

	#define time_division_factor 100
	
#elif defined(fiftieth_ms_division)

	#define time_division_factor 20

#endif


/************************************************************************/



/* USER CODE END Private defines */

void MX_TIM2_Init(void);

/* USER CODE BEGIN Prototypes */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

