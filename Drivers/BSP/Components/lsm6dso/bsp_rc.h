#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"
#include "main.h"

extern void RC_Init(uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
#endif
