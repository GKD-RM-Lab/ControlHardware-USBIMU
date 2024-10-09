#include "EKF_fusion.hpp"
#include "LSM6DSO_Task.hpp"

void EKF_fusion_Task(void *argument)
{
    while (1)
    {
        IMU.print_data();
        vTaskDelay(1);
    }
    
}

EKF_fusion::EKF_fusion(/* args */)
{
}

EKF_fusion::~EKF_fusion()
{
}