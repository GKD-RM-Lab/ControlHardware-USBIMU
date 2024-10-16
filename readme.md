## 项目简介
一个USB陀螺仪模块的固件，IMU选用LSM6DSO，地磁仪为MMC5603，MCU型号是STM32G431CBU6，为rm-controls电控方案设计。另外还能控制四路PWM舵机和一路红点激光的开关。

角速度、加速度、地磁向量融合为欧拉角的步骤在MCU上完成，使用MotionFx库。

## 构建说明

环境依赖为：

- gcc-arm-none-eabi
- openocd
- make
- (可选) STM32CubeMX

先使用makefile构建项目：

```sh
make -j6
```

再使用openocd烧录：

```sh
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program build/RMCONTROL-G431.bin 0x08000000 verify" -c "reset run" -c "exit"
```

## 通讯协议

数据包格式和通讯协议定义如下：

通过USB虚拟串口与NUC通讯，包含的功能有：

- 主动TX：
    - 默认以1kHz的频率向NUC回报IMU数据，欧拉角形式
- 被动TX：
    - ~~NUC发出询问信号时回复当前硬件ID和固件版本~~（TODO 还没写完）
- RX：
    - 接收NUC的命令，调整四个通道的PWM占空比，以及红点激光开关
    - 接收NUC的命令，调整IMU回报频率

### 数据包规定

1. `uint8_t` 帧头: 0x5A
2. `uint8_t` 命令ID
3. `uint8_t` 当前数据包长度（包含头尾）
4. `<数据段>`
5. `uint8_t` 帧尾: 0x9C

存在的命令ID：

```cpp
#define CMD_IMU_TX 0x01          // IMU数据发送帧
#define CMD_IMU_SET_FREQ 0x02    // IMU数据发送频率设置
#define CMD_PWM_SET 0x03         // PWM占空比设置
#define CMD_LASER_SET 0x04       // 激光开关设置
#define CMD_HARDWARE_INFO 0x05   // 硬件信息查询
```

### 结构体定义

```cpp
/* IMU发送数据包结构体定义 */
typedef struct
{
    uint8_t head = FRAME_HEAD;  // 帧头
    uint8_t cmd = CMD_IMU_TX;   // 命令ID
    uint8_t length;             // 数据长度
    float yaw;
    float pitch;
    float roll;
    uint8_t end = FRAME_TAIL;   // 帧尾
} __attribute__((packed)) Frame_type;
Frame_type frame;
```

```cpp
/* 设定IMU回报频率数据包结构体 */
typedef struct
{
    uint8_t head;    // 帧头
    uint8_t cmd;     // 命令ID
    uint8_t length;
    uint16_t freq;   // 频率（单位Hz）
    uint8_t end;     // 帧尾
} __attribute__((packed)) Frame_freq_type;
```

```cpp
/* 设定四个通道PWM占空比的数据包结构体 */
typedef struct
{
    uint8_t head;    // 帧头
    uint8_t cmd;     // 命令ID
    uint8_t length;
    uint16_t pwm[4]; // 四个通道的PWM占空比
    uint8_t end;     // 帧尾
} __attribute__((packed)) Frame_pwm_type;
```

```cpp
/* 设定红点激光开关的数据包结构体 */
typedef struct
{
    uint8_t head;    // 帧头
    uint8_t cmd;     // 命令ID
    uint8_t length;
    uint8_t laser;   // 激光开关
    uint8_t end;     // 帧尾
} __attribute__((packed)) Frame_laser_type;
```

## 文件结构

主要功能使用 C++ 完成，位于 `./application` 中：

- `LSM6DSO_Task.cpp`：负责维护和读取 LSM6DSO 和 MMC5603 的加速度计、角速度和地磁仪数据。
- `EKF_fusion.cpp`：负责将读取到的 9 轴传感器数据通过 EKF 融合成稳定的欧拉角。
    - STM32G431 在168MHZ下运行一次 EKF 融合需要 600us，理论上回报率可以达到 1.6kHz，默认 IMU 回报率为 1kHz。
- `USB_VCP_Task.cpp`：负责维护 USB 虚拟串口。

进程调度位于 `Core/Src/app_freertos.c` 中。

修改了 `main.c`，可以通过读取地址 GPIO 的电平来设置 USB 设备的 PID。

## 调度结构

开机时，初始化完 GPIO 外设后会立刻读取拨码开关来确定 USB 设备的 PID，然后初始化 USB 和其他设备。

主要使用 RTOS 调度，开启了以下线程：

- **EKF_fusion_Task**：读取 IMU 和地磁仪原始数据，并通过 EKF 融合得到欧拉角，使用 `vTaskDelayUntil` 方法保证精准的执行周期。
- **USB_TX_TASK_Handle**：定时使用 USB 虚拟串口发送 IMU 数据。

RTOS 线程的创建位于 `Core/Src/app_freertos.c`。

USB 虚拟串口的接收使用中断调度，原始回调函数位于 `USB_Device/App/usbd_cdc_if.c` 的 `CDC_Receive_FS`，并重定向到 `application/USB_VCP_Task.cpp` 中。

需要注意的是，FreeRTOS的的SystemTick频率被修改成了10000hz，所以`vTaskDelay(1)`对应的是`0.1ms`而不是常规的`1ms`。

## TODO
- 地磁仪的读取和EKF融合
  - 先考虑MMC5603的兼容
  - 后续添加ITS8301的兼容（因为MMC5603难焊）
- 完善USB虚拟串口的通讯&指令执行
  - 固定周期回报陀螺仪数据 √
  - 切换IMU回报率
  - 设定舵机pwm占空比
  - 设定红点激光开关
  - 回报硬件信息
- WS2812 RBG LED
  - 使用DMA PWM控制WS2812 LED存在一些问题，地磁仪调好之后再来看看（