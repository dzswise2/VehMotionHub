/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "app_rtos.h"
#include "app_tasks.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

osThreadId defaultTaskHandle;

void MX_FREERTOS_Init(void);

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  if (ppxIdleTaskTCBBuffer != NULL)
  {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  }
  if (ppxIdleTaskStackBuffer != NULL)
  {
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
  }
  if (pulIdleTaskStackSize != NULL)
  {
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  }
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

osThreadId dataOutTaskHandle;
osThreadId canComTaskHandle;
osThreadId workTaskHandle;

osThreadDef(dataOutTask, data_output, osPriorityNormal, 1, 512);
osThreadDef(canComTask, can_com, osPriorityAboveNormal, 1, 512);
osThreadDef(workTask, work_state, osPriorityBelowNormal, 1, 256);

void MX_FREERTOS_Init(void)
{
  App_RtosInitObjects();

  dataOutTaskHandle = osThreadCreate(osThread(dataOutTask), NULL);
  canComTaskHandle = osThreadCreate(osThread(canComTask), NULL);
  workTaskHandle = osThreadCreate(osThread(workTask), NULL);

  if ((dataOutTaskHandle == NULL) ||
      (canComTaskHandle == NULL) ||
      (workTaskHandle == NULL))
  {
    Error_Handler();
  }
}
