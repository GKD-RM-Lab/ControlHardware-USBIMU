#ifndef USB_Task_H
#define USB_Task_H

/**********************************
 * 通过USB虚拟串口与NUC通讯，包含的功能有：
 * 主动TX：
 * 默认以1khz的频率向nuc回报imu数据，欧拉角形式
 * 
 * 被动TX：
 * nuc发出询问信号的时候回复当前硬件ID和固件版本
 * 
 * RX：
 * 接收nuc的命令，调整四个通道的pwm占空比，以及红点激光开关
 * 接收nuc的命令，调整IMU回报频率
 * （收到命令后，反馈命令执行情况）
 * 
 * 数据包规定
 * 1. uint8_t 帧头: 0x5A
 * 2. uint8_t 命令ID
 * 3. uint8_t 当前数据包长度(包含头尾)
 * 4. <数据段>
 * 5. uint8_t 帧尾: 0x9C
 * 
 **********************************/

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
//usb vcp driver
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "main.h"

/*帧命令ID定义*/
#define FRAME_HEAD 0x5A;            //帧头
#define FRAME_TAIL 0x9C;            //帧尾

#define CMD_IMU_TX 0x01;            //IMU数据发送帧
#define CMD_IMU_SET_FREQ 0x02;      //IMU数据发送频率设置
#define CMD_PWM_SET 0x03;           //PWM占空比设置
#define CMD_LASER_SET 0x04;         //激光开关设置
#define CMD_HARDWARE_INFO 0x05;     //硬件信息查询

/*IMU发送数据包结构体定义*/
typedef struct
{
    uint8_t head = FRAME_HEAD;              //帧头
    uint8_t cmd = CMD_IMU_TX;               //命令ID
    uint8_t length;                         //数据长度, 之后初始化，根据数据长度用sizeof计算
    float yaw;
    float pitch;
    float roll;
    uint8_t end = FRAME_TAIL;               //帧尾
}__attribute__((packed)) Frame_type;
Frame_type frame;

/*设定IMU回报频率数据包结构体*/
typedef struct
{
    uint8_t head;        //帧头
    uint8_t cmd;        //命令ID
    uint8_t length;
    uint16_t freq;       //频率(单位Hz)
    uint8_t end;        //帧尾
}__attribute__((packed)) Frame_freq_type;

/*设定四个通道PWM占空比的数据包结构体*/
typedef struct
{
    uint8_t head;        //帧头
    uint8_t cmd;        //命令ID
    uint8_t length;
    uint16_t pwm[4];    //四个通道的PWM占空比
    uint8_t end;        //帧尾
}__attribute__((packed)) Frame_pwm_type;

/*设定红点激光开关的数据包结构体*/
typedef struct
{
    uint8_t head;        //帧头
    uint8_t cmd;        //命令ID
    uint8_t length;
    uint8_t laser;      //激光开关
    uint8_t end;        //帧尾
}__attribute__((packed)) Frame_laser_type;

/*硬件信息查询的数据包结构体*/
typedef struct
{
    uint8_t head;        //帧头
    uint8_t cmd;        //命令ID
    uint8_t length;
    uint8_t hardware_id;    //硬件ID
    uint8_t firmware_version; //固件版本
    uint8_t end;        //帧尾
}__attribute__((packed)) Frame_hardware_info_type;


class USB_VCPTask
{
private:
    /* data */
public:
    void instruct_decode(uint8_t *buf, uint8_t len);    //处理接收到的数据
    void imu_angle_send(float *angle, float *angle_v);
    void imu_angle_send_vofa(float *angle, float *angle_v);
    
};


#endif  // __cplusplus


/*FreeRTOS的调用接口*/
#ifdef __cplusplus
extern "C" {
#endif

void USB_VCP_RX_Task(void *argument);
void USB_VCP_TX_Task(void *argument);
void CDC_Receive_CallBack(uint8_t* pbuf, uint32_t *Len);

#ifdef __cplusplus
}
#endif

#endif