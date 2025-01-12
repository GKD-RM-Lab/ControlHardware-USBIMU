/*#include "RC_Task.h"


uint8_t rxDataBuffer[50];
uint32_t rxData;
uint8_t rxDataLen;


void RC_Task(void *argument)
{
     __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);

    HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rxDataBuffer, sizeof(rxDataBuffer));

    while(1)
    {
        cprintf(&huart3, "task_ok\n");
        vTaskDelay(1);
    }
}


void HAL_UARTEx_RxEventCa11back(UART_HandleTypeDef  *huart, uint16_t Size)
{
    if(huart==&hlpuart1)
    {
        rxData = *((uint32_t *)rxDataBuffer);
        rxDataLen = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rxDataBuffer, sizeof(rxDataBuffer));

        cprintf(&huart3, "666\n");
    }
}
*/


#include "RC_Task.h"


void RC_Task(void *argument)
{

    while(1)
    {


        cprintf(&huart3, "task_ok\n");
        vTaskDelay(1);
    }
}



