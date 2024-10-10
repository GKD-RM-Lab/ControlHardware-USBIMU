#include "LSM6DSO_Task.hpp"

LSM6DSO_Handle IMU;

void LSM6DSO_Task(void *argument)
{
    IMU.begin();
    cprintf(&huart3, "IMU id = %x\n", IMU.checkid());
    cprintf(&huart3, "temperature:%d\n", (int)IMU.get_temperature());
    /*以1KHZ的频率轮询获取数据*/
    while (1)
    {
        // if(!IMU.ready()) continue;
        IMU.update();
        // IMU.print_data();
        // IMU.plot_data();
        vTaskDelay(2);      //周期0.3ms，采样率约为3khz

    }
    
}

void LSM6DSO_Handle::plot_data()
{
    cprintf(&huart3, "%d,%d,%d,%d,%d,%d,%d\n", (int)IMU.angular_rate_mdps[0],
                    (int)IMU.angular_rate_mdps[1], (int)IMU.angular_rate_mdps[2],
                    (int)IMU.acceleration_mg[0], (int)IMU.acceleration_mg[1],
                    (int)IMU.acceleration_mg[2], (int)IMU.temperature_degC);
}

void LSM6DSO_Handle::print_data()
{
    cprintf(&huart3, "Gyro:%d, %d, %d(mdps)\n", (int)IMU.angular_rate_mdps[0],
                    (int)IMU.angular_rate_mdps[1], (int)IMU.angular_rate_mdps[2]);
    cprintf(&huart3, "Accel:%d, %d, %d(mg)\n", (int)IMU.acceleration_mg[0],
                    (int)IMU.acceleration_mg[1], (int)IMU.acceleration_mg[2]);
    cprintf(&huart3, "temperature:%d\n", (int)IMU.temperature_degC);
}

/*更新LSM6DSO数据*/
void LSM6DSO_Handle::update()
{
    /*加速度计数据*/
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lsm6dso_acceleration_raw_get(&reg_ctx, data_raw_acceleration);
    acceleration_mg[0] =
    lsm6dso_from_fs4_to_mg(data_raw_acceleration[0]);
    acceleration_mg[1] =
    lsm6dso_from_fs4_to_mg(data_raw_acceleration[1]);
    acceleration_mg[2] =
    lsm6dso_from_fs4_to_mg(data_raw_acceleration[2]);
    
    /*角速度计数据*/
    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
    lsm6dso_angular_rate_raw_get(&reg_ctx, data_raw_angular_rate);
    angular_rate_mdps[0] =
    lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[0]);
    angular_rate_mdps[1] =
    lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[1]);
    angular_rate_mdps[2] =
    lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[2]);

    /*温度计数据*/
    temperature_degC = IMU.get_temperature();

}

/*判断数据是否就位*/
int8_t LSM6DSO_Handle::ready()
{
    uint8_t data_ready;
    lsm6dso_temp_flag_data_ready_get(&reg_ctx, &data_ready);
    return data_ready;
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
    /* Disable I3C interface */
    lsm6dso_i3c_disable_set(&reg_ctx, LSM6DSO_I3C_DISABLE);
    /* Enable Block Data Update */
    lsm6dso_block_data_update_set(&reg_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lsm6dso_xl_data_rate_set(&reg_ctx, xl_odr_set);
    lsm6dso_gy_data_rate_set(&reg_ctx, gyro_odr_set);
    /* Set full scale */
    lsm6dso_xl_full_scale_set(&reg_ctx, xl_fullscale_set);
    lsm6dso_gy_full_scale_set(&reg_ctx, gyro_fullscale_set);
    /* Configure filtering chain(No aux interface)
    * Accelerometer - LPF1 + LPF2 path
    */
    lsm6dso_xl_hp_path_on_out_set(&reg_ctx, LSM6DSO_HP_PATH_DISABLE_ON_OUT);
    lsm6dso_xl_filter_lp2_set(&reg_ctx, PROPERTY_ENABLE);
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
int32_t SPI2_IOSend(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg, 1, 1000);
    HAL_SPI_Transmit(&hspi2, (uint8_t*) bufp, len, 1000);
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    return 0;
}

int32_t SPI2_IORecv(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    reg |= 0x80;
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg, 1, 1000);
    HAL_SPI_Receive(&hspi2, bufp, len, 1000);
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    return 0;
}

void freertos_delay(uint32_t ms)
{
    vTaskDelay(ms);
}