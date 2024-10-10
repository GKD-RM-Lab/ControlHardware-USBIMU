#include "USB_VCP_Task.hpp"
#include "EKF_fusion.hpp"
#include "LSM6DSO_Task.hpp"

USB_VCPTask Usb;

/*USB接收线程*/
void USB_VCP_RX_Task(void *argument)
{
    while (1)
    {
        cprintf(&huart3, "usb rx ok\n");
        vTaskDelay(1000);
    }
    
}

/*USB发送线程*/
void USB_VCP_TX_Task(void *argument)
{
    while (1)
    {
        Usb.imu_angle_send(EKF.Angle_fused);
        vTaskDelay(10);         //1khz

        // EKF.plot_angle();
        // IMU.plot_data();
        // EKF.print_angle();
    }
    
}

void USB_VCPTask::cdc_rx_handler(uint8_t *buf, uint8_t len)
{
    // switch (buf[0])
    // {
    // case CMD_IMU_SET_FREQ:
    //     break;
    // case CMD_PWM_SET:
    //     break;
    // case CMD_LASER_SET:
    //     break;
    // case CMD_HARDWARE_INFO:
    //     break;
    // default:
    //     break;
    // }
}

void USB_VCPTask::imu_angle_send(float *angle)
{
    frame.yaw = angle[0];
    frame.pitch = angle[1];
    frame.roll = angle[2];
    frame.length = sizeof(frame);
    CDC_Transmit_FS((uint8_t *)&frame, sizeof(frame));
}