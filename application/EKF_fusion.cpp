#include "EKF_fusion.hpp"
#include "LSM6DSO_Task.hpp"

// EKF_fusion EKF;

void EKF_fusion_Task(void *argument)
{

    while (1)
    {
        // EKF.caculate(IMU.acceleration_mg, IMU.angular_rate_mdps);
        // IMU.print_data();
        // EKF.print_angle();
        // cprintf(&huart3, "--->%d\n", (int)(EKF.delta_time * 1000));
        // vTaskDelay((int)(EKF.delta_time * 1000));   //按照EKF计算周期延时
        cprintf(&huart3, "ok\n");
    }
    
}
//输出EKF融合之后的欧拉角
void EKF_fusion::print_angle()
{
    cprintf(&huart3, "Yaw:%d, Pitch:%d, Roll:%d\n", Angle_fused[0],
                    Angle_fused[1], Angle_fused[2]);
}   
//计算EKF
void EKF_fusion::caculate(float *acceleration_mg, float *angular_rate_mdps)
{
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
    for(int i=0; i<2; i++){
        Angle_fused[i] = data_out.rotation[i];
    }
        
}

/*初始化EKF*/
EKF_fusion::EKF_fusion(/* args */)
{
    __CRC_CLK_ENABLE();
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

EKF_fusion::~EKF_fusion()
{
}