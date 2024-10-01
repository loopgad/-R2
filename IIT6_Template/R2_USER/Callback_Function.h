#ifndef Callback_Function
#define Callback_Function

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"
#include "Global_Namespace.h"
#include <cstdint>

using namespace Remote_Namespace;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif // Callback_Function