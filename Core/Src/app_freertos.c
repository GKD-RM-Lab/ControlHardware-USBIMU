/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
LSM6D璇诲彇鐩稿叧
鍏堟祴璇曪紝鍚庢湡鍐嶆暣鐞�
*/
#include "custom_mems_conf.h"
#include "lsm6dso_reg.h"
#include "lsm6dso.h"
#include "com.h"
#include "usart.h"
#include "spi.h"

/*妯″潡澶存枃浠�*/
#include "LSM6DSO_Task.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t LEDTaskHandle;
osThreadId_t LSM6DSOTR_IMU_Handle;

const osThreadAttr_t Lite_Task_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256
};

const osThreadAttr_t Heavy_Task_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/***************IMU Task***************/
osThreadId_t LSM6DSO_TASK_Handle;
const osThreadAttr_t LSM6DSO_TASK_attributes = {
  .name = "lsm6dso task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024
};
/* USER CODE END FunctionPrototypes */

void LED_Task(void *argument);
void LSM6DSOTR_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //LSM6DSO 澶勭悊绾跨▼
  LSM6DSO_TASK_Handle = osThreadNew(LSM6DSO_Task, NULL, &LSM6DSO_TASK_attributes);
  
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/************************************************************************************************
LSM6D璇诲彇鐩稿叧
鍏堟祴璇曪紝鍚庢湡鍐嶆暣鐞�
************************************************************************************************/

int cs_en = 1;
/*榛樿�や紶鎰熷櫒閰嶇疆*/
lsm6dso_fs_xl_t acc_scale = LSM6DSO_4g;
lsm6dso_fs_g_t gyro_scale = LSM6DSO_500dps;
lsm6dso_odr_xl_t acc_odr = LSM6DSO_XL_ODR_104Hz;
lsm6dso_bdr_xl_t acc_batch_odr = LSM6DSO_XL_BATCHED_AT_104Hz;
lsm6dso_odr_g_t gyro_odr = LSM6DSO_GY_ODR_104Hz;
lsm6dso_bdr_gy_t gyro_batch_odr = LSM6DSO_GY_BATCHED_AT_104Hz;
lsm6dso_shub_odr_t mag_sh_odr = LSM6DSO_SH_ODR_104Hz;
int32_t fifo_size = 30;

// SPI鍙戦€佹暟鎹�鍖呰�呭櫒鍑芥暟
int32_t SPI2_Send(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    if (Length > 128) {
        return -1;
    }

    uint8_t data[128 + 1];  
    data[0] = (reg | 0x80);            // 璁剧疆瀵勫瓨鍣ㄥ湴鍧€涓虹��涓€涓�瀛楄妭
    memcpy(&data[1], pData, Length);   // 澶嶅埗鏁版嵁

    int err = 0;
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    err = BSP_SPI2_Send(data, Length + 1);
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);

    return err;
}

// SPI鎺ユ敹鏁版嵁鍖呰�呭櫒鍑芥暟
int32_t SPI2_Recv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    uint8_t dataReg = reg | 0x80; // 璁剧疆璇讳綅锛屽亣璁綧SB鏄�璇诲啓鎺у埗浣�
    
    // 鍙戦€佸瘎瀛樺櫒鍦板潃
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    BSP_SPI2_Send(&dataReg, 1);  // 鍏堝彂閫佸瘎瀛樺櫒鍦板潃
    
    // 鎺ユ敹鏁版嵁
    int err = 0;
    err = BSP_SPI2_Recv(pData, Length);
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    return err;
}

/*LSM6D IO 缁撴瀯浣�*/
stmdev_ctx_t reg_ctx;


/*鐢ㄦ潵瀛樻斁璇诲彇鍒扮殑鏁版嵁*/
float angular_rate_mdps[3];
float acceleration_mg[3];
float magnetic_mG[3];

void lsm6dso_begin(){


}

/*璇诲彇LSM6D鏁版嵁*/
void Lsm6dso_Sensor_update() {
    uint16_t num = 114;
    lsm6dso_fifo_tag_t reg_tag;
    // timestamp_sample_t ts_tick;
    uint32_t timestamp = 0.0f;
    // axis3bit16_t dummy;

    /* get sample sum */
    lsm6dso_fifo_data_level_get(&reg_ctx, &num);
    cprintf(&huart3, "%d\n", num);

    // TODO: since f411 should be always catch up?
    /* we should always try to catch up the latest sample */
    if (num > 0) {
        
        while (num--) {
            /* Read FIFO tag. */
            lsm6dso_fifo_sensor_tag_get(&reg_ctx, &reg_tag);

            switch (reg_tag) {
                case LSM6DSO_XL_NC_TAG:
                    // memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
                    // lsm6dso_fifo_out_raw_get(&reg_ctx, data_raw_acceleration.u8bit);
                    // acceleration_mg[0] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[0]);
                    // acceleration_mg[1] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[1]);
                    // acceleration_mg[2] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[2]);
                    break;

                case LSM6DSO_GYRO_NC_TAG:
                    // memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
                    // lsm6dso_fifo_out_raw_get(&reg_ctx, data_raw_angular_rate.u8bit);
                    // angular_rate_mdps[0] = lsm6dso_from_fs500_to_mdps(data_raw_angular_rate.i16bit[0]);
                    // angular_rate_mdps[1] = lsm6dso_from_fs500_to_mdps(data_raw_angular_rate.i16bit[1]);
                    // angular_rate_mdps[2] = lsm6dso_from_fs500_to_mdps(data_raw_angular_rate.i16bit[2]);
                    break;

                case LSM6DSO_SENSORHUB_SLAVE0_TAG:
                    // memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
                    // lsm6dso_fifo_out_raw_get(&reg_ctx, data_raw_magnetic.u8bit);
	            
                    // TODO: check & debug
                    // magnetic_mG[0] = normalizeRawMag(data_raw_magnetic.i16bit[0]);
                    // magnetic_mG[1] = normalizeRawMag(data_raw_magnetic.i16bit[1]);
                    // magnetic_mG[2] = normalizeRawMag(data_raw_magnetic.i16bit[2]);
                    break;

                // case LSM6DSO_TIMESTAMP_TAG:
                //     lsm6dso_fifo_out_raw_get(&reg_ctx, ts_tick.byte);
                //     timestamp = (unsigned int)lsm6dso_from_lsb_to_nsec(ts_tick.reg.tick);
                //     break;

                default:
                    /* Flush unused samples. */
                    // memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
                    // lsm6dso_fifo_out_raw_get(&reg_ctx, dummy.u8bit);
                    break;
            }

            // Got sample
            // if (has_new_acc && has_new_gyro && has_new_mag) {
            // if (has_new_acc && has_new_gyro) {
            //     break;
            // }

        }

    }

}


void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  if (BSP_SPI2_Init() != HAL_OK) {
      cprintf(&huart3, "SPI2 INIT ERROR");
  }
  // LSM6D 椹卞姩缁撴瀯浣撻厤缃�
  reg_ctx.write_reg = SPI2_Send;
  reg_ctx.read_reg = SPI2_Recv;
  reg_ctx.handle = &hspi2;

  // 鍒濆�嬪寲LSM6DSO浼犳劅鍣�
  LSM6DSO_Object_t lsm6dso_obj;
  lsm6dso_obj.Ctx = reg_ctx;

  //灏濊瘯璇诲彇ID
  uint8_t id = 0x00;
  if (LSM6DSO_ReadID(&lsm6dso_obj, &id) == 0) {
    cprintf(&huart3, "id read ok, id = %x\n", id);
    if(id == 0x6c) cprintf(&huart3, "ID OK\n");
    else cprintf(&huart3, "ID NOT OK\n");
  }else{
    cprintf(&huart3, "id read fail\n");
  }
  
  /*閰嶇疆LSM6D*/
  lsm6dso_i3c_disable_set(&reg_ctx, LSM6DSO_I3C_DISABLE);

  int err = 0;
  err = LSM6DSO_GYRO_Enable(&lsm6dso_obj);
  cprintf(&huart3, "gyro init err = %d\n", err);
  
  LSM6DSO_GYRO_SetOutputDataRate(&lsm6dso_obj, LSM6DSO_XL_ODR_833Hz);  // 璁剧疆ODR
  LSM6DSO_GYRO_Set_Power_Mode(&lsm6dso_obj, LSM6DSO_GY_HIGH_PERFORMANCE);  // 璁剧疆楂樻€ц兘妯″紡
  LSM6DSO_GYRO_SetFullScale(&lsm6dso_obj, LSM6DSO_250dps);  // 璁剧疆闄€铻轰华鐨勯噺绋�
  
  for(;;)
  {
    LSM6DSO_Axes_t gyro_data;
    LSM6DSO_GYRO_GetAxes(&lsm6dso_obj, &gyro_data);
    cprintf(&huart3, "Gyro X: %d, Y: %d, Z: %d\n", gyro_data.x, gyro_data.y, gyro_data.z);
    LSM6DSO_ReadID(&lsm6dso_obj, &id);
    cprintf(&huart3, "id=%x\n", id);
    vTaskDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/************************************************************************************************
 * end of ism6dso test
************************************************************************************************/


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

