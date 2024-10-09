#include "EKF_fusion.hpp"
#include "LSM6DSO_Task.hpp"

EKF_fusion EKF;

void EKF_fusion_Task(void *argument)
{
    while (1)
    {
        EKF.caculate(IMU.acceleration_mg, IMU.angular_rate_mdps);
        IMU.print_data();
        EKF.print_angle();
        cprintf(&huart3, "--->%d\n", (int)(EKF.delta_time * 1000));
        vTaskDelay((int)(EKF.delta_time * 1000));   //按照EKF计算周期延时
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

EKF_fusion::EKF_fusion(/* args */)
{
}

EKF_fusion::~EKF_fusion()
{
}