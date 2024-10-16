#include "LED_Task.hpp"

uint8_t led_buffer[LED_COUNT * 24]; // 每个LED 24位
int8_t set_ws2812(uint8_t red, uint8_t green, uint8_t blue, int led_index);

void LED_Task(void *argument)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 100);
    while(1)
    {
        // set_ws2812(255, 0, 0, 0);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        
    }
}

int8_t set_ws2812(uint8_t red, uint8_t green, uint8_t blue, int led_index)
{
    int i;
    for (i = 0; i < 8; i++) {
        led_buffer[led_index * 24 + i] = (green & (1 << (7 - i))) ? WS2812_HIGH : WS2812_LOW;
        led_buffer[led_index * 24 + 8 + i] = (red & (1 << (7 - i))) ? WS2812_HIGH : WS2812_LOW;
        led_buffer[led_index * 24 + 16 + i] = (blue & (1 << (7 - i))) ? WS2812_HIGH : WS2812_LOW;
    }
    // 使用DMA启动传输
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)led_buffer, LED_COUNT * 24);
}
