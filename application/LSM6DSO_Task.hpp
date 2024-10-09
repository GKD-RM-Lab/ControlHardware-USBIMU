#ifndef LSM6DSO_Task_H
#define LSM6DSO_Task_H

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
//lsm6dso driver
#include "custom_mems_conf.h"
#include "lsm6dso_reg.h"
#include "lsm6dso.h"
#include "spi.h"

class LSM6DSO_Handle
{
private:
public:
    stmdev_ctx_t reg_ctx;
    LSM6DSO_Object_t lsm6dso_obj;

    uint8_t checkid();
    LSM6DSO_Handle(/* args */);
    ~LSM6DSO_Handle();
};




#endif  // __cplusplus


/*FreeRTOS的调用接口*/
#ifdef __cplusplus
extern "C" {
#endif

void LSM6DSO_Task(void *argument);
int32_t SPI2_IOSend(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length);
int32_t SPI2_IORecv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif