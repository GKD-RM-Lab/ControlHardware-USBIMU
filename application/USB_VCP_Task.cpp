#include "USB_VCP_Task.hpp"
#include "EKF_fusion.hpp"
#include "LSM6DSO_Task.hpp"

void USB_VCP_Task(void *argument)
{
    while (1)
    {
        // cprintf(&huart3, "USB Task OK\n");
        // EKF.plot_angle();
        IMU.plot_data();
        // EKF.print_angle();
        vTaskDelay(10*10);  //100hz
    }
    
}