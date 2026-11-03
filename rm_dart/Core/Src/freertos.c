/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ins_task.h"
#include "ctl_task.h"
#include "spi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Ins_Task */
osThreadId_t Ins_TaskHandle;
const osThreadAttr_t Ins_Task_attributes = {
  .name = "Ins_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CTL_Task */
osThreadId_t CTL_TaskHandle;
const osThreadAttr_t CTL_Task_attributes = {
  .name = "CTL_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED_Task */
osThreadId_t LED_TaskHandle;
const osThreadAttr_t LED_Task_attributes = {
  .name = "LED_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void IMU_Task(void *argument);
void Control_Task(void *argument);
void led_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Ins_Task */
  Ins_TaskHandle = osThreadNew(IMU_Task, NULL, &Ins_Task_attributes);

  /* creation of CTL_Task */
  CTL_TaskHandle = osThreadNew(Control_Task, NULL, &CTL_Task_attributes);

  /* creation of LED_Task */
  LED_TaskHandle = osThreadNew(led_task, NULL, &LED_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the Ins_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
  uint8_t imu_error;

  control_notify_imu_ready(0U);
  do
  {
    /* Recalibrate gyro bias while the dart is stationary at every power-up.
       Yaw has no gravity reference, so a stale Z bias integrates directly. */
    imu_error = BMI088_init(&hspi1, 1U);
    if (imu_error != BMI088_NO_ERROR)
      osDelay(100U);
  } while (imu_error != BMI088_NO_ERROR);

  INS_Init();
  osDelay(1U);
  INS_Task();
  control_notify_imu_ready(1U);
  /* Infinite loop */
  for(;;)
  {
    INS_Task();
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the CTL_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
void Control_Task(void *argument)
{
  /* USER CODE BEGIN Control_Task */
	control_Init();
  /* Infinite loop */
  for(;;)
  {
    control_Task();
    osDelay(DART_CONTROL_PERIOD_MS);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
void led_task(void *argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
    led_Task();
    /* 50 Hz 向 OpenMV 发送 pitch/yaw/roll 姿态帧。
       放在低优先级任务中，避免阻塞 2 ms 控制任务。 */
    attitude_telemetry_send();
    /* 握手成功前每 100 ms 发送握手帧；收到 ACK 后停止。 */
    openmv_handshake_send();
    osDelay(20U);
  }
  /* USER CODE END led_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

