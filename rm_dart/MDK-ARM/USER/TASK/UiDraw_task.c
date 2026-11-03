#include "UiDraw_task.h"
//#include "Tasks.h"


#include "priority_ui.h"
//#include "drive.h"
#include "ui.h"

float com=0; 

void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
		com++;
		
	  
    osDelay(1);
  }
  /* USER CODE END StartControlTask */
}



/**
  * @Name    UiDraw_task
  * @brief   实时任务更新
  * @param   
  * @retval 
  * @author  
  * @Date    
**/
void UiDraw_task(void const * argument)
{
  /* USER CODE BEGIN UiDraw_task */
  /* Infinite loop */
  for(;;)
  {
		com++;
		

    osDelay(1);
  }
  /* USER CODE END UiDraw_task */
}



