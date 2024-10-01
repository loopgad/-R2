#include "Hardware_Peripheral.h"

//reset action module(with toggle switch)
void Hardware_Peripheral::Action_Reset(void) {
    const char *str = "ACT0";
    while (*str) {
        HAL_UART_Transmit(&huart3, (uint8_t *)str++, 1, HAL_MAX_DELAY);
    }
}


void Hardware_Peripheral::Air_Pump_Control(uint16_t buf[10])//control the pumps to push the ball
{
	
	//put the two pumps into the same state
	if(buf[6]>1500){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
	}

	
}