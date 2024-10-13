#ifndef LED_Task_H
#define LED_Task_H


/*CPP功能部分*/
#ifdef __cplusplus
//RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//debug serial
#include "com.h"
#include "usart.h"
//ws2812
#include "tim.h"
#include "dma.h"
#include "gpio.h"

#define LED_COUNT 10  // 控制的LED数量
#define WS2812_HIGH 60 // 高电平时间，单位：timer ticks
#define WS2812_LOW 30  // 低电平时间，单位：timer ticks

class WS2812_LED
{
private:
    uint8_t led_buffer[LED_COUNT * 24]; // 每个LED 24位
public:
    int8_t set(uint8_t red, uint8_t green, uint8_t blue, int led_index);   //设置led颜色
    WS2812_LED(/* args */);
};





#endif  // __cplusplus


/*FreeRTOS的调用接口*/
#ifdef __cplusplus
extern "C" {
#endif

void LED_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif