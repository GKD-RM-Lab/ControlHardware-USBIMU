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
    /*默认传感器配置*/
    lsm6dso_fs_xl_t acc_scale = LSM6DSO_4g;
    lsm6dso_fs_g_t gyro_scale = LSM6DSO_500dps;
    lsm6dso_odr_xl_t acc_odr = LSM6DSO_XL_ODR_104Hz;
    lsm6dso_bdr_xl_t acc_batch_odr = LSM6DSO_XL_BATCHED_AT_104Hz;
    lsm6dso_odr_g_t gyro_odr = LSM6DSO_GY_ODR_104Hz;
    lsm6dso_bdr_gy_t gyro_batch_odr = LSM6DSO_GY_BATCHED_AT_104Hz;
    lsm6dso_shub_odr_t mag_sh_odr = LSM6DSO_SH_ODR_104Hz;
    int32_t fifo_size = 30;
public:
    /*设备对象*/
    stmdev_ctx_t reg_ctx;
    LSM6DSO_Object_t lsm6dso_obj;
    /*传感器数据*/
    static int16_t data_raw_acceleration[3];
    static int16_t data_raw_angular_rate[3];
    static int16_t data_raw_temperature;
    static float acceleration_mg[3];
    static float angular_rate_mdps[3];
    static float temperature_degC;
    static uint8_t whoamI, rst;
    
    /*工具函数*/
    float_t get_temperature();      //获取温度
    void begin();                   //开始采样
    void reset();                   //重置
    uint8_t checkid();              //读取IMU ID
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
static void freertos_delay(uint32_t ms);
#ifdef __cplusplus
}
#endif

#endif