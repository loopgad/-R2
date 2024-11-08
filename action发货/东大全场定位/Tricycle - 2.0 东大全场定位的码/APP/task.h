#ifndef __TASK_H
#define __TASK_H
#include "sys.h"

extern uint8_t Bsp_Int_Ok; 


void Nvic_Init(void); 
void BSP_Init(void);
void Task_1000HZ(void);
void Task_500HZ(void);
void Task_250HZ(void);
void Task_200HZ(void);
void Task_100HZ(void);
void Task_50HZ(void);
void Task_20HZ(void);
void Task_10HZ(void);
void Task_1HZ(void);

#endif
