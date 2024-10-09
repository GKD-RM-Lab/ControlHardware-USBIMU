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

class EKF_fusion
{
private:
    int16_t ALGO_FREQ = 1000U;    //刷新率
    int16_t ALGO_PERIOD = (1000U / ALGO_FREQ);
public:
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