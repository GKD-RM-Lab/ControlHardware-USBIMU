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
LSM6D读取相关
先测试，后期再整理
*/
#include "custom_mems_conf.h"
#include "lsm6dso_reg.h"
#include "lsm6dso.h"
#include "com.h"
#include "usart.h"
#include "spi.h"

/*模块头文件*/
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
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
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

void StartDefaultTask(void *argument);

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

  //LSM6DSO 处理线程
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
LSM6D读取相关
先测试，后期再整理
************************************************************************************************/

int cs_en = 1;
/*默认传感器配置*/
lsm6dso_fs_xl_t acc_scale = LSM6DSO_4g;
lsm6dso_fs_g_t gyro_scale = LSM6DSO_500dps;
lsm6dso_odr_xl_t acc_odr = LSM6DSO_XL_ODR_104Hz;
lsm6dso_bdr_xl_t acc_batch_odr = LSM6DSO_XL_BATCHED_AT_104Hz;
lsm6dso_odr_g_t gyro_odr = LSM6DSO_GY_ODR_104Hz;
lsm6dso_bdr_gy_t gyro_batch_odr = LSM6DSO_GY_BATCHED_AT_104Hz;

lsm6dso_shub_odr_t mag_sh_odr = LSM6DSO_SH_ODR_104Hz;

int32_t fifo_size = 30;

// SPI发送数据包装器函数
int32_t SPI2_Send(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    if (Length > 128) {
        return -1;
    }

    uint8_t data[128 + 1];  
    data[0] = (reg | 0x80);            // 设置寄存器地址为第一个字节
    memcpy(&data[1], pData, Length);   // 复制数据

    int err = 0;
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    err = BSP_SPI2_Send(data, Length + 1);
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);

    return err;
}

// SPI接收数据包装器函数
int32_t SPI2_Recv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    uint8_t dataReg = reg | 0x80; // 设置读位，假设MSB是读写控制位
    
    // 发送寄存器地址
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    BSP_SPI2_Send(&dataReg, 1);  // 先发送寄存器地址
    
    // 接收数据
    int err = 0;
    err = BSP_SPI2_Recv(pData, Length);
    if(cs_en) HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    return err;
}

/*LSM6D IO 结构体*/
stmdev_ctx_t reg_ctx;


/*用来存放读取到的数据*/
float angular_rate_mdps[3];
float acceleration_mg[3];
float magnetic_mG[3];

void lsm6dso_begin(){


}

/*读取LSM6D数据*/
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
  // LSM6D 驱动结构体配置
  reg_ctx.write_reg = SPI2_Send;
  reg_ctx.read_reg = SPI2_Recv;
  reg_ctx.handle = &hspi2;

  // 初始化LSM6DSO传感器
  LSM6DSO_Object_t lsm6dso_obj;
  lsm6dso_obj.Ctx = reg_ctx;

  //尝试读取ID
  uint8_t id = 0x00;
  if (LSM6DSO_ReadID(&lsm6dso_obj, &id) == 0) {
    cprintf(&huart3, "id read ok, id = %x\n", id);
    if(id == 0x6c) cprintf(&huart3, "ID OK\n");
    else cprintf(&huart3, "ID NOT OK\n");
  }else{
    cprintf(&huart3, "id read fail\n");
  }
  
  /*配置LSM6D*/
  lsm6dso_i3c_disable_set(&reg_ctx, LSM6DSO_I3C_DISABLE);

  int err = 0;
  err = LSM6DSO_GYRO_Enable(&lsm6dso_obj);
  cprintf(&huart3, "gyro init err = %d\n", err);
  
  LSM6DSO_GYRO_SetOutputDataRate(&lsm6dso_obj, LSM6DSO_XL_ODR_833Hz);  // 设置ODR
  LSM6DSO_GYRO_Set_Power_Mode(&lsm6dso_obj, LSM6DSO_GY_HIGH_PERFORMANCE);  // 设置高性能模式
  LSM6DSO_GYRO_SetFullScale(&lsm6dso_obj, LSM6DSO_250dps);  // 设置陀螺仪的量程
  
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

