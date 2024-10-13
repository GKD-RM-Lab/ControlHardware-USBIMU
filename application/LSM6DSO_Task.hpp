#ifndef LSM6DSO_Task_H
#define LSM6DSO_Task_H

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
//sh driver
#include "sh_mag.h"

/*CPP功能部分*/
#ifdef __cplusplus

typedef union {
    int16_t i16bit[3]; // 16位访问
    uint8_t u8bit[6];  // 字节访问
} axis3bit16_t;

typedef union {
    int32_t i32bit;
    uint8_t u8bit[4];  
} time_stamp;


/***** IMU类 *****/
class LSM6DSO_Handle : Lsm6d_sh_mag
{
private:
    /*默认传感器配置*/
    lsm6dso_odr_xl_t xl_odr_set = LSM6DSO_XL_ODR_6667Hz;    //加速度计采样率
    lsm6dso_fs_xl_t xl_fullscale_set = LSM6DSO_4g;          //加速度计最大量程
    lsm6dso_odr_g_t gyro_odr_set = LSM6DSO_GY_ODR_6667Hz;   //陀螺仪采样率
    lsm6dso_fs_g_t gyro_fullscale_set = LSM6DSO_2000dps;    //陀螺仪最大量程
public:
    /*设备对象*/
    stmdev_ctx_t reg_ctx;
    LSM6DSO_Object_t lsm6dso_obj;
    /*传感器数据*/
    axis3bit16_t data_raw_acceleration{};
    axis3bit16_t data_raw_angular_rate{};
    axis3bit16_t data_raw_magnetic{};
    int16_t data_raw_temperature;
    float acceleration_mg[3];
    float angular_rate_mdps[3];
    float magnetic_mG[3]{};
    float temperature_degC;
    uint8_t whoamI, rst;
    time_stamp this_timestamp;    //数据时间戳
    time_stamp last_timestamp;
    
    /* SH收发 */
    uint8_t SH_IO_Write(uint8_t *buf, uint8_t reg, uint16_t num) override;
    uint8_t SH_IO_Read(uint8_t *buf, uint8_t reg, uint16_t num) override;

    /* 工具函数 */
    void plot_data();               //输出便于vofa+显示的数据
    void print_data();              //输出当前存储的数据
    void update();                  //更新数据
    float_t get_temperature();      //获取温度
    int8_t ready();                 //新数据可读
    void begin();                   //开始采样
    void reset();                   //重置
    uint8_t checkid();              //读取IMU ID
    LSM6DSO_Handle(/* args */);
    ~LSM6DSO_Handle();
};

extern LSM6DSO_Handle IMU;

#endif  // __cplusplus


/*FreeRTOS的调用接口*/
#ifdef __cplusplus
extern "C" {
#endif

void LSM6DSO_Task(void *argument);
int32_t SPI2_IOSend(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length);
int32_t SPI2_IORecv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length);
void freertos_delay(uint32_t ms);
#ifdef __cplusplus
}
#endif

#endif