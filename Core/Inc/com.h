#ifndef __COM_H__
#define __COM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "ctype.h"
	
#include "stm32g4xx_hal.h"

extern int cprintf(void *p, const char *format, ...);
	
extern char cgetc(void *p);
	
extern void cgets(void *p, char *str, char end);
	
extern void cgets_s(void *p, char *str, int s);
	
extern void cgets(void *p, char *str, char end);

extern void receiveInit(UART_HandleTypeDef *p, void (*f)(const char *));

extern void receiveSet(UART_HandleTypeDef *p, void (*f)(const char *));

extern void receiveStart(UART_HandleTypeDef *huart, int sz);

#ifdef __cplusplus
}
#endif

#endif
