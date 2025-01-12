#ifndef RC_TASK_H
#define RC_TASK_H

//RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//debug serial
#include "com.h"
#include "usart.h"

void RC_Task(void *argument);

#endif