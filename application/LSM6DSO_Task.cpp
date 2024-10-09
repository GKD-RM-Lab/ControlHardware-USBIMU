#include "LSM6DSO_Task.hpp"


LSM6DSO_Handle IMU;

void LSM6DSO_Task(void *argument)
{
    IMU.begin();
    while (1)
    {
        cprintf(&huart3, "id = %x\n", IMU.checkid());

        LSM6DSO_AxesRaw_t gyro_data;
        // LSM6DSO_GYRO_GetAxesRaw(&IMU.lsm6dso_obj, &gyro_data);
        // cprintf(&huart3, "Gyro X: %d, Y: %d, Z: %d\n", gyro_data.x, gyro_data.y, gyro_data.z);
        cprintf(&huart3, "temperature:%d\n", IMU.get_temperature() * 1000);

        vTaskDelay(100);
    }
    
}

/*读取IMU温度传感器*/
float_t LSM6DSO_Handle::get_temperature()
{
    float temperature_degC;
    static int16_t data_raw_temperature;
    memset(&data_raw_temperature, 0x00, sizeof(int16_t));
    lsm6dso_temperature_raw_get(&reg_ctx, &data_raw_temperature);
    temperature_degC =
        lsm6dso_from_lsb_to_celsius(data_raw_temperature);
    return temperature_degC;
}


/*读取LSM6DSO ID*/
//return 1: error
uint8_t LSM6DSO_Handle::checkid()
{
    u_int8_t id=0x00;
    if (LSM6DSO_ReadID(&lsm6dso_obj, &id) != 0) {
        return 1;
    }
    return id;
}

/*IMU 参数初始化&采样开始*/
void LSM6DSO_Handle::begin()
{
    LSM6DSO_Handle::reset();
    /*禁用IMU*/
    lsm6dso_i3c_disable_set(&reg_ctx, LSM6DSO_I3C_DISABLE);
    /*配置陀螺仪*/
    LSM6DSO_GYRO_Enable(&lsm6dso_obj);
    LSM6DSO_GYRO_SetOutputDataRate(&lsm6dso_obj, LSM6DSO_XL_ODR_833Hz);  // 设置ODR
    LSM6DSO_GYRO_Set_Power_Mode(&lsm6dso_obj, LSM6DSO_GY_HIGH_PERFORMANCE);  // 设置高性能模式
    LSM6DSO_GYRO_SetFullScale(&lsm6dso_obj, LSM6DSO_250dps);  // 设置陀螺仪的量程
}


void LSM6DSO_Handle::reset()
{
    uint8_t rst;

    /* Restore default configuration */
    lsm6dso_reset_set(&reg_ctx, PROPERTY_ENABLE);

    do {
        lsm6dso_reset_get(&reg_ctx, &rst);
    } while (rst);
}

/*IMU SPI初始化*/
LSM6DSO_Handle::LSM6DSO_Handle(/* args */)
{
    BSP_SPI2_Init();
    reg_ctx.write_reg = SPI2_IOSend;
    reg_ctx.read_reg = SPI2_IORecv;
    reg_ctx.mdelay = freertos_delay;
    reg_ctx.handle = &hspi2;
    lsm6dso_obj.Ctx = reg_ctx;
}

LSM6DSO_Handle::~LSM6DSO_Handle()
{
}


/********************SPI IO收发********************/
int32_t SPI2_IOSend(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    if (Length > 128) {
        return -1;
    }

    uint8_t data[128 + 1];  
    data[0] = (reg | 0x80);            // 设置寄存器地址为第一个字节
    memcpy(&data[1], pData, Length);   // 复制数据

    int err = 0;
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    err = BSP_SPI2_Send(data, Length + 1);
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);

    return err;
}

int32_t SPI2_IORecv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    uint8_t dataReg = reg | 0x80; // 设置读位，假设MSB是读写控制位
    
    // 发送寄存器地址
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    BSP_SPI2_Send(&dataReg, 1);  // 先发送寄存器地址
    
    // 接收数据
    int err = 0;
    err = BSP_SPI2_Recv(pData, Length);
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    return err;
}

static void freertos_delay(uint32_t ms)
{
    vTaskDelay(ms);
}