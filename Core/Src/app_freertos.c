/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/*应用调用*/
#include "LSM6DSO_Task.hpp"
#include "EKF_fusion.hpp"
#include "USB_VCP_Task.hpp"
#include "LED_Task.hpp"

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
/* Definitions for defaultTask */
osThreadId_t LSM6DSOTR_IMU_Handle;

const osThreadAttr_t Lite_Task_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256
};

const osThreadAttr_t Heavy_Task_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/***************IMU Task***************/
osThreadId_t LSM6DSO_TASK_Handle;
const osThreadAttr_t LSM6DSO_TASK_attributes = {
  .name = "lsm6dso task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024
};
/***************EKF Task***************/
osThreadId_t EKF_TASK_Handle;
const osThreadAttr_t EKF_TASK_attributes = {
  .name = "EKF task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/***************USB RX Task***************/
//低优先级 但不主动释放资源 在cpu空闲的时候处理接受缓冲区中的数据
osThreadId_t USB_RX_TASK_Handle;
const osThreadAttr_t USB_RX_TASK_attributes = {
  .name = "USB RX task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1024
};
/***************USB TX Task***************/
//高优先级 1khz频率运行
osThreadId_t USB_TX_TASK_Handle;
const osThreadAttr_t USB_TX_TASK_attributes = {
  .name = "USB TX task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024
};
/***************LEDTask***************/
osThreadId_t LED_TASK_Handle;
const osThreadAttr_t LED_TASK_attributes = {
  .name = "LED task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024
};
/* USER CODE END FunctionPrototypes */


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
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //LSM6DSO Task
  // LSM6DSO_TASK_Handle = osThreadNew(LSM6DSO_Task, NULL, &LSM6DSO_TASK_attributes);
  //EKF Task
  EKF_TASK_Handle = osThreadNew(EKF_fusion_Task, NULL, &EKF_TASK_attributes);
  //USB TX Task
  USB_TX_TASK_Handle = osThreadNew(USB_VCP_TX_Task, NULL, &USB_TX_TASK_attributes);
  //USB RX Task
  // USB_RX_TASK_Handle = osThreadNew(USB_VCP_RX_Task, NULL, &USB_RX_TASK_attributes);  
  //LED Task
  LED_TASK_Handle = osThreadNew(LED_Task, NULL , &LED_TASK_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */



void StartDefaultTask(void *argument)
{

  for(;;)
  {
    vTaskDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/************************************************************************************************
 * end of ism6dso test
************************************************************************************************/


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

