#ifndef EKF_FUSION_Task_H
#define EKF_FUSION_Task_H


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
//EKF library
#include "motion_fx.h"

/*单位换算宏*/
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f


class EKF_fusion
{
private:
    /*基本参数*/
    MFX_input_t data_in;
    MFX_output_t data_out;
    int fusion_flag = 0;
    uint8_t mfxstate[2432];
public:
    float delta_time = 0.001;    //EKF计算周期(ms)

    float Angle_fused[3];   //EFK融合输出的数据 {yaw, pitch, roll}
    void print_angle();     //输出EKF融合之后的欧拉角
    void caculate(float *acceleration_mg, float *angular_rate_mdps);        //计算EKF
    EKF_fusion(/* args */);
    ~EKF_fusion();
};



#endif  // __cplusplus


/*FreeRTOS的调用接口*/
#ifdef __cplusplus
extern "C" {
#endif

void EKF_fusion_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif