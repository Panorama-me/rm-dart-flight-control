#ifndef __TASKS_H
#define __TASKS_H

#include "cmsis_os.h"


/******************Task******************/

#include "Correspondence_Task.h"
#include "led_flow_task.h"
#include "UiDraw_task.h"
#include "ins_task.h"
/******************Task******************/

void UiDraw_task(void const * argument);

//extern correspondence_ctrl Corres;
#ifdef __cplusplus
extern "C" {
#endif

void Correspondence_Task(void const * argument);
void led_flow_task(void const * argument);
void StartControlTask(void const * argument);
	
	
#ifdef __cplusplus
}
#endif



#endif



