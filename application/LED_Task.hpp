#ifndef LED_Task_H
#define LED_Task_H

#define LED_COUNT 10  // 控制的LED数量
#define WS2812_HIGH 200 // 高电平时间，单位：timer ticks
#define WS2812_LOW 100  // 低电平时间，单位：timer ticks

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
#include "main.h"




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