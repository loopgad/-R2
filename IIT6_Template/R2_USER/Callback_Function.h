#ifndef Callback_Function
#define Callback_Function

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "stm32h7xx_hal.h"
#include "ROS.h"
#include "Global_Namespace.h"
#include <cstdint>
#include <string.h>

using namespace Remote_Namespace; 
using namespace Action_Namespace;

//
#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0
#define PI 3.14159265358979323846

#define   robot_start 0
#define   robot_load_left 1
#define	  robot_shoot 2
#define   robot_load_right 3

#define y  		0
#define x 		1
#define w		2

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Update_Action(float value[6]);

//#ifdef __cplusplus
//}
//#endif

#endif // Callback_Function