/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
#define CAN_RxExtId 0x0000
#define CAN_TxExtId 0x0000
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN1_Filter_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN2_Filter_Init(void);

extern CAN_TxHeaderTypeDef	TxHeader;      


// 瀹氫箟涓?涓粨鏋勪綋 latestCAN1_Message锛岀敤浜庡瓨�?? CAN 娑堟伅鐨勬渶鏂版暟鎹?
typedef struct {
    CAN_RxHeaderTypeDef RxHeader;  // CAN 娑堟伅澶?
    uint8_t RxData[8];             // CAN 娑堟伅鏁版嵁 (8 瀛楄�?)
} CAN_RX_Message_t;

extern CAN_RX_Message_t latestCAN1_Message;//鐢ㄤ簬涓篗otor_update鎻愪緵鎺ュ彛
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

