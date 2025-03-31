/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "math.h"
/* USER CODE BEGIN 0 */

uint8_t rx_buffer[1] = {0};
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
LaserProcessor laser;  // 全局对象
uint32_t Encoder1;
float Angle_encoder = 360.0f/(4096*3)/3*9.1f/3.75f, Plate_angle1;
void encoder2_processing_data(unsigned char ucData)
{
    static unsigned char ucRxBuffer1[20];
    static unsigned char ucRxCnt1 = 0;

    ucRxBuffer1[ucRxCnt1++] = ucData;  // 将收到的数据存入缓冲区中

    if( ucRxBuffer1[0] != 0xAB || (ucRxBuffer1[1] != 0xCD && ucRxCnt1 > 1)) // 数据头不对，则重新开始寻找0x55数据头(数据头固定不变)
    {
        ucRxCnt1 = 0;
        return;
    }
    if(ucRxCnt1 < 10) {return;}  // 数据不满11个，则返回
    else
    {
        Encoder1 = ucRxBuffer1[5] ;
        Encoder1 = Encoder1<<8 ;
        Encoder1 |= ucRxBuffer1[6] ;
        Encoder1 = Encoder1<<8 ;
        Encoder1 |= ucRxBuffer1[3] ;
        Encoder1 = Encoder1<<8 ;
        Encoder1 |= ucRxBuffer1[4] ;

//        if(Encoder1 > 65535/2 )
//            Plate_angle1 = (Encoder1 * Angle_encoder - 65535 * Angle_encoder)-5760+1100;//
//        else
//            Plate_angle1 = Encoder1 * Angle_encoder;


            Plate_angle1 = (float)(Encoder1%4096) / 4096 * 360;//

		
        ucRxCnt1 = 0;  // 清空缓存区
    }
}
// 串口中断回调函数
uint32_t delta_time = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
		encoder2_processing_data((unsigned char)rx_buffer[0]);
		HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    }
}
/* USER CODE END 1 */
