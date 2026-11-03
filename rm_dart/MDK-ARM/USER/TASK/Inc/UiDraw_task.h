#ifndef __UIDRAW_TASK_H
#define __UIDRAW_TASK_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"



#ifdef __cplusplus
extern "C" {
#endif

void StartControlTask(void const * argument);
void UiDraw_task(void const * argument);
	
	
#ifdef __cplusplus
}
#endif









#endif














