// #include <iostream>
#include "LED_TASK.hpp"


void LED_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    vTaskDelay(1000);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    vTaskDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}
