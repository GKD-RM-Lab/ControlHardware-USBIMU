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

/*EKF参数宏*/
#define STATE_SIZE                      (size_t)(2432)

#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)

#define DECIMATION                      1U

class EKF_fusion
{
private:
    /*基本参数*/
    MFX_input_t data_in;
    MFX_output_t data_out;
    int fusion_flag = 0;
    uint8_t mfxstate[2432];
    MFX_knobs_t iKnobs;
    MFX_knobs_t *ipKnobs = &iKnobs;
public:
    float delta_time = 0.001;    //EKF计算周期(ms)
    uint32_t elapsed_time_us;    //上一次EFK计算耗时(us)

    /*EKF输出数据*/
    float Angle_fused[3];   //EFK融合输出的数据 {yaw, pitch, roll}
    float Quaternion[4];
    float linear_acceleration[3];
    
    void print_angle();     //输出EKF融合之后的欧拉角
    void plot_angle();
    void init();            //EKF初始化
    void caculate(float *acceleration_mg, float *angular_rate_mdps);        //计算EKF
};

extern EKF_fusion EKF;

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