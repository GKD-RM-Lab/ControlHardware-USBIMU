#include "bsp_rc.h"
#include "main.h"
#include "usart.h"
#include "stm32g4xx_hal_uart.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern volatile uint8_t sbus_rx_buf[200];

 
void RC_Init(uint16_t dma_buf_num)
{
    /*
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart2,(uint8_t *)sbus_rx_buf,200);

    SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
*/
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buf, 200);

    __HAL_DMA_DISABLE(&hdma_usart2_rx);

    while(hdma_usart2_rx.Instance->CCR & DMA_CCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart2_rx);
    }

    __HAL_DMA_ENABLE(&hdma_usart2_rx);
}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart2);
}
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart2);
    __HAL_DMA_DISABLE(&hdma_usart2_rx);

    hdma_usart2_rx.Instance->CNDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart2_rx);
    __HAL_UART_ENABLE(&huart2);

}

