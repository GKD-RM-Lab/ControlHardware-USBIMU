#ifndef LED_TASK_H
#define LED_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void LED_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
