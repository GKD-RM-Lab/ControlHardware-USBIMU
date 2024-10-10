#ifndef USB_Task_H
#define USB_Task_H

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
//usb vcp driver



#endif  // __cplusplus


/*FreeRTOS的调用接口*/
#ifdef __cplusplus
extern "C" {
#endif

void USB_VCP_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif