#include "EKF_fusion.hpp"
#include "LSM6DSO_Task.hpp"


/*******用vTaskDelayUntil确保精确的执行周期********/
EKF_fusion EKF;
void EKF_fusion_Task(void *argument)
{
    EKF.delta_time = 0.0012;    // 周期设置为1.2ms
    EKF.init();
    IMU.begin();

    vTaskDelay(1000);        //等待IMU初始化完成

    //跳过开机时的零数据
    while(1)
    {
        IMU.update();
        if(IMU.acceleration_mg[0] != 0) break;
        vTaskDelay(100);
    }

    //记录当前时间 & 执行周期转换成system tick
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = EKF.delta_time * 1000 * 10;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        IMU.update();

        EKF.caculate(IMU.acceleration_mg, IMU.angular_rate_mdps);
        
        // 使用vTaskDelayUntil来确保固定周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // IMU.print_data();
        // EKF.print_angle();
        // cprintf(&huart3, "--->%d\n", (int)(EKF.delta_time * 1000 * 10));
    }
}

//输出EKF融合之后的欧拉角，直接输出包含浮点的字节流，适配vofa+显示
void EKF_fusion::plot_angle()
{
    typedef struct
    {
        float yaw;
        float pitch;
        float roll;
        uint8_t tail[4]{0x00, 0x00, 0x80, 0x7f};
    }__attribute__((packed)) Frame_type;
    Frame_type frame;

    frame.yaw = Angle_fused[0];
    frame.pitch = Angle_fused[1];
    frame.roll = Angle_fused[2];

    HAL_UART_Transmit(&huart3, (uint8_t *)&frame, sizeof(frame), HAL_MAX_DELAY);    

} 

//输出EKF融合之后的欧拉角
void EKF_fusion::print_angle()
{
    int factor = 10;
    cprintf(&huart3, "Yaw:%d, Pitch:%d, Roll:%d, time:%dus\n", (int)(Angle_fused[0]*factor),
                    (int)(Angle_fused[1]*factor), (int)(Angle_fused[2]*factor), elapsed_time_us);
    
} 

/*计算EKF*/
void EKF_fusion::caculate(float *acceleration_mg, float *angular_rate_mdps)
{
    //耗时统计 begin
    TickType_t start_tick, end_tick;
    start_tick = xTaskGetTickCount();

    // 读取IMU类中已有的传感器数据
    data_in.gyro[0] = IMU.angular_rate_mdps[0] * FROM_MDPS_TO_DPS;
    data_in.gyro[1] = IMU.angular_rate_mdps[1] * FROM_MDPS_TO_DPS;
    data_in.gyro[2] = IMU.angular_rate_mdps[2] * FROM_MDPS_TO_DPS;

    data_in.acc[0] = IMU.acceleration_mg[0] * FROM_MG_TO_G;
    data_in.acc[1] = IMU.acceleration_mg[1] * FROM_MG_TO_G;
    data_in.acc[2] = IMU.acceleration_mg[2] * FROM_MG_TO_G;
    
    // 不使用磁力计
    data_in.mag[0] = 0.0f;
    data_in.mag[1] = 0.0f;
    data_in.mag[2] = 0.0f;

    //计算EKF
    MotionFX_propagate(mfxstate, &data_out, &data_in, &delta_time);
    MotionFX_update(mfxstate, &data_out, &data_in, &delta_time, NULL);

    //输出欧拉角
    for(int i=0; i<3; i++){
        Angle_fused[i] = data_out.rotation[i];
    }

    //耗时统计 end
    end_tick = xTaskGetTickCount();
    elapsed_time_us = (end_tick - start_tick) * 100;
        
}

/*初始化EKF*/
void EKF_fusion::init()
{
    __CRC_CLK_ENABLE();     //确保CRC开启
    MotionFX_initialize((MFXState_t *)mfxstate);
    MotionFX_getKnobs(mfxstate, ipKnobs);

    ipKnobs->acc_orientation[0] = 's';
    ipKnobs->acc_orientation[1] = 'e';
    ipKnobs->acc_orientation[2] = 'u';
    ipKnobs->gyro_orientation[0] = 's';
    ipKnobs->gyro_orientation[1] = 'e';
    ipKnobs->gyro_orientation[2] = 'u';

    ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
    ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;

    ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
    ipKnobs->LMode = 1;
    ipKnobs->modx = DECIMATION;

    MotionFX_setKnobs(mfxstate, ipKnobs);
    MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
    MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);
}

