/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os2.h"
#include "niming.h"
#include "usart.h"
#include "OLED.h"
#include "QD4310.h"
#include "icm42688.h"
#include <math.h>       // 用于 sqrt, atan2, cos, sin
#include <string.h>     // 用于 memset
#include "imu_filter.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* 按键引脚定义 */
#define KEY1_PORT   GPIOC
#define KEY1_PIN    GPIO_PIN_8
#define KEY2_PORT   GPIOC
#define KEY2_PIN    GPIO_PIN_7
/* USER CODE BEGIN PTD */
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} IMU_Data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_Data_t;

// // Mahony 滤波器结构体
// typedef struct {
//     float q0, q1, q2, q3; // 四元数
//     float exInt, eyInt, ezInt; // 误差积分
//     float Kp, Ki;         // 增益参数
// } Mahony_Filter_t;

// // 简单 PID 结构体
// typedef struct {
//     float Kp, Ki, Kd;
//     float integral;
//     float last_error;
// } PID_TypeDef;
extern QD4310_t Motor_0;
extern QD4310_t Motor_1;
extern QD4310_t Motor_2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 电机PID输出（给CAN任务用）
// extern float motor0_out = 0.0f;
// extern float motor1_out = 0.0f;
// extern float motor2_out = 0.0f;

// === 无停止二次校准（在闭环中采集） ===
static uint8_t gyro_recalib_done = 0;
static uint32_t stable_start_time = 0;
static uint8_t calib_state = 0;      // 0:空闲, 1:采集中
static float calib_sum_gx = 0, calib_sum_gy = 0, calib_sum_gz = 0;
static uint16_t calib_count = 0;
#define CALIB_SAMPLE_NUM    500      // 采样次数（约200ms）
#define CALIB_GYRO_THRESH   2.0f     // 角速度阈值 dps
#define STABLE_THRESHOLD_DEG  0.5f     // 误差小于0.5°视为稳定
#define STABLE_DURATION_MS    2000     // 稳定持续5秒后触发校准
// 陀螺仪校准偏移
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

/* USER CODE END Variables */
/* Definitions for LED_Task */
osThreadId_t LED_TaskHandle;
const osThreadAttr_t LED_Task_attributes = {
  .name = "LED_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for OLED_Task */
osThreadId_t OLED_TaskHandle;
const osThreadAttr_t OLED_Task_attributes = {
  .name = "OLED_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for CAN_Task */
osThreadId_t CAN_TaskHandle;
const osThreadAttr_t CAN_Task_attributes = {
  .name = "CAN_Task",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for Motor_Control_T */
osThreadId_t Motor_Control_THandle;
const osThreadAttr_t Motor_Control_T_attributes = {
  .name = "Motor_Control_T",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for Attitude_Task */
osThreadId_t Attitude_TaskHandle;
const osThreadAttr_t Attitude_Task_attributes = {
  .name = "Attitude_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for KEY_Task */
osThreadId_t KEY_TaskHandle;
const osThreadAttr_t KEY_Task_attributes = {
  .name = "KEY_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for IMU_Data_Queue */
osMessageQueueId_t IMU_Data_QueueHandle;
const osMessageQueueAttr_t IMU_Data_Queue_attributes = {
  .name = "IMU_Data_Queue"
};
/* Definitions for Attitude_Data_Queue */
osMessageQueueId_t Attitude_Data_QueueHandle;
const osMessageQueueAttr_t Attitude_Data_Queue_attributes = {
  .name = "Attitude_Data_Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLED_Task(void *argument);
void StartOLED_Task(void *argument);
void StartUART_Task(void *argument);
void StartCAN_Task(void *argument);
void StartIMU_Task(void *argument);
void StartMotor_Control_Task(void *argument);
void StartAttitude_Task(void *argument);
void StartKEY_Task(void *argument);

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

  /* Create the queue(s) */
  /* creation of IMU_Data_Queue */
  IMU_Data_QueueHandle = osMessageQueueNew (16, sizeof(IMU_Data_t), &IMU_Data_Queue_attributes);

  /* creation of Attitude_Data_Queue */
  Attitude_Data_QueueHandle = osMessageQueueNew (16, sizeof(Attitude_Data_t), &Attitude_Data_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LED_Task */
  LED_TaskHandle = osThreadNew(StartLED_Task, NULL, &LED_Task_attributes);

  /* creation of OLED_Task */
  OLED_TaskHandle = osThreadNew(StartOLED_Task, NULL, &OLED_Task_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(StartUART_Task, NULL, &UART_Task_attributes);

  /* creation of CAN_Task */
  CAN_TaskHandle = osThreadNew(StartCAN_Task, NULL, &CAN_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(StartIMU_Task, NULL, &IMU_Task_attributes);

  /* creation of Motor_Control_T */
  Motor_Control_THandle = osThreadNew(StartMotor_Control_Task, NULL, &Motor_Control_T_attributes);

  /* creation of Attitude_Task */
  Attitude_TaskHandle = osThreadNew(StartAttitude_Task, NULL, &Attitude_Task_attributes);

  /* creation of KEY_Task */
  KEY_TaskHandle = osThreadNew(StartKEY_Task, NULL, &KEY_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLED_Task */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLED_Task */
void StartLED_Task(void *argument)
{
  /* USER CODE BEGIN StartLED_Task */
  /* Infinite loop */
  for(;;)
  {
  uint32_t led_tick = 0;
  const uint32_t LED_PERIOD_MS = 5000;
    for(;;)
    {
      osDelay(100);                  // 100ms 周期
      led_tick += 100;

      if (led_tick >= LED_PERIOD_MS)
      {
        led_tick = 0;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
      }
    }  
  }
  /* USER CODE END StartLED_Task */
}

/* USER CODE BEGIN Header_StartOLED_Task */
/**
* @brief Function implementing the OLED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLED_Task */
void StartOLED_Task(void *argument)
{
  /* USER CODE BEGIN StartOLED_Task */
  OLED_Init();

  /* Infinite loop */
  for(;;)
  {
  osDelay(100);

  OLED_Clear();

  OLED_ShowString(0,0,"R:",OLED_8X16);
  OLED_ShowFloatNum(16,0,euler.roll,2,1,OLED_8X16);

  OLED_ShowString(0,16,"P:",OLED_8X16);
  OLED_ShowFloatNum(16,16,euler.pitch,2,1,OLED_8X16);

  OLED_ShowString(0,32,"Y:",OLED_8X16);
  OLED_ShowFloatNum(16,32,euler.yaw,2,1,OLED_8X16);
  // 刷新屏幕
  OLED_Update();
  }
  /* USER CODE END StartOLED_Task */
}

/* USER CODE BEGIN Header_StartUART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_Task */
void StartUART_Task(void *argument)
{
  /* USER CODE BEGIN StartUART_Task */
  /* Infinite loop */
  for(;;)
  {
  Get_NMdata( euler.roll ,euler.pitch,euler.yaw);
  HAL_UART_Transmit(&huart1, BUFF, sizeof(BUFF), 10);  
  osDelay(20);
  }
  /* USER CODE END StartUART_Task */
}

/* USER CODE BEGIN Header_StartCAN_Task */
/**
* @brief Function implementing the CAN_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN_Task */
void StartCAN_Task(void *argument)
{
  /* USER CODE BEGIN StartCAN_Task */

//   motor1_out=55;
  /* Infinite loop */
  for(;;)
  {
  osDelay(200);


  }
  /* USER CODE END StartCAN_Task */
}

/* USER CODE BEGIN Header_StartIMU_Task */
//IMU校准函数
void Gyro_Calibrate(void)
{
    float sumgx=0, sumgy=0, sumgz=0;
    IMU_Data_t temp;
    for(int i=0;i<500;i++)
    {
        ICM42688_Read_All(&temp.ax,&temp.ay,&temp.az,&temp.gx,&temp.gy,&temp.gz);
        sumgx += temp.gx;
        sumgy += temp.gy;
        sumgz += temp.gz;
        osDelay(1);
    }
    gx_offset = sumgx/500;
    gy_offset = sumgy/500;
    gz_offset = sumgz/500;
}


/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMU_Task */
void StartIMU_Task(void *argument)
{
  /* USER CODE BEGIN StartIMU_Task */
  /* Infinite loop */
  IMU_Data_t imu_data;
  // 初始化IMU
  ICM42688_Init();
  Gyro_Calibrate();
  for(;;)
  {
    // 读取IMU数据
 if (ICM42688_Read_All(&imu_data.ax, &imu_data.ay, &imu_data.az,
                          &imu_data.gx, &imu_data.gy, &imu_data.gz) == 0)
    {
        // 只有读取成功才发送队列，避免发送垃圾数据
        osMessageQueuePut(IMU_Data_QueueHandle, &imu_data, 0, 0);
    }

    osDelay(1);
  }
  /* USER CODE END StartIMU_Task */
}

/* USER CODE BEGIN Header_StartMotor_Control_Task */
/**
* @brief Function implementing the Motor_Control_T thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotor_Control_Task */
void StartMotor_Control_Task(void *argument)
{
  /* USER CODE BEGIN StartMotor_Control_Task */
  osDelay(500);
  Motor_Init_All();
  osDelay(100) ;
  /* Infinite loop */
  for(;;)
  {
// 1. 执行平衡解算 (计算 motor0/1/2_out)
  Motor_Balance_Control();

  // 2. 执行硬件限位保护 (修正 motor1/2_out)
  Motor_Speed_Limit_All();

  // 3. 发送最终指令给电机驱动
  // 注意：这里直接调用 QD4310_SetSpeed，因为我们已经优化了驱动为非阻塞
  QD4310_SetSpeed(&Motor_0, motor0_out);
  QD4310_SetSpeed(&Motor_1, motor1_out);
  QD4310_SetSpeed(&Motor_2, motor2_out);

  // 4. 精确延时，控制回路频率
  // osDelay(2) 意味着 500Hz 的控制频率
  osDelay(2);
  }
  /* USER CODE END StartMotor_Control_Task */
}

/* USER CODE BEGIN Header_StartAttitude_Task */
/**
* @brief Function implementing the Attitude_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAttitude_Task */
void StartAttitude_Task(void *argument)
{
  /* USER CODE BEGIN StartAttitude_Task */
  // osDelay(1000); // 等待系统稳定
  Mahony_Init(15.0f, 0.0f);
  IMU_Data_t imu;
  for(;;)
  {
      if(osMessageQueueGet(IMU_Data_QueueHandle, &imu, 0, osWaitForever) == osOK)
  {
        // 1. 减偏移
        float gx_raw = imu.gx - gx_offset;
        float gy_raw = imu.gy - gy_offset;
        float gz_raw = imu.gz - gz_offset;

        // ======================

        // ======================
        float gx = gx_raw / 16.4f;
        float gy = gy_raw / 16.4f;
        float gz = gz_raw / 8.2f; //出于某种未知原因输出的raw差了一倍，所以直接在这里加倍了

        // 姿态解算
        Mahony_Update(gx, gy, gz, imu.ax, imu.ay, imu.az);

        Quat_To_Euler();
        euler.roll=-euler.roll;
        euler.roll += 180.0f;
        if (euler.roll > 180.0f) euler.roll -= 360.0f;
        if (euler.roll < -180.0f) euler.roll += 360.0f;
        euler.pitch = -euler.pitch;
        
        // Yaw 处理：加 180 度并归一化到 -180~180
        euler.yaw += 180.0f;
        if (euler.yaw > 180.0f) euler.yaw -= 360.0f;
        if (euler.yaw < -180.0f) euler.yaw += 360.0f;

          
        // ========== 二次校准（无停电机） ==========
        if (Balance_Mode_Enable && !gyro_recalib_done)
        {
            // 阶段1：稳定检测
            if (calib_state == 0)
            {
                float err_roll  = fabsf(euler.roll - Target_Roll_Angle);

                float err_pitch = fabsf(euler.pitch - Target_Pitch_Angle);

                if (err_roll < STABLE_THRESHOLD_DEG && err_pitch < STABLE_THRESHOLD_DEG)
                {
                    if (stable_start_time == 0)
                        stable_start_time = osKernelGetTickCount();
                    else if ((osKernelGetTickCount() - stable_start_time) >= STABLE_DURATION_MS)
                    {
                        // 进入采集状态
                        calib_state = 1;
                        calib_sum_gx = calib_sum_gy = calib_sum_gz = 0;
                        calib_count = 0;
                        stable_start_time = 0;
                    }
                }
                else
                {
                    stable_start_time = 0;
                }
            }
            // 阶段2：采集原始陀螺仪数据（减当前偏移后仍接近0）
            else if (calib_state == 1)
            {
                // 获取原始物理值（dps），未经过偏移修正
                float raw_gx = imu.gx;   // ICM42688_Read_All 已经转为 dps，存在 imu.gx 中
                float raw_gy = imu.gy;
                float raw_gz = imu.gz;
                
                // 检查角速度是否足够小（说明云台基本静止）
                if (fabsf(raw_gx) < CALIB_GYRO_THRESH &&
                    fabsf(raw_gy) < CALIB_GYRO_THRESH &&
                    fabsf(raw_gz) < CALIB_GYRO_THRESH)
                {
                    calib_sum_gx += raw_gx;
                    calib_sum_gy += raw_gy;
                    calib_sum_gz += raw_gz;
                    calib_count++;
                }
                
                if (calib_count >= CALIB_SAMPLE_NUM)
                {
                    // 计算新的偏移量（取平均）
                    float new_gx_off = calib_sum_gx / CALIB_SAMPLE_NUM;
                    float new_gy_off = calib_sum_gy / CALIB_SAMPLE_NUM;
                    float new_gz_off = calib_sum_gz / CALIB_SAMPLE_NUM;
                    
                    // 更新全局偏移
                    gx_offset = new_gx_off;
                    gy_offset = new_gy_off;
                    gz_offset = new_gz_off;
                    
                    // 重置 Mahony 滤波器（清除累积误差）
                    Mahony_Init(15.0f, 0.0f);
                    
                    // 标记完成
                    gyro_recalib_done = 1;
                    calib_state = 0;
                    
                    // 串口输出提示
                    char msg[] = "\r\nGyro recalibrated without motor stop!\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
                }
            }
          }
      }
  }
  /* USER CODE END StartAttitude_Task */
}

/* USER CODE BEGIN Header_StartKEY_Task */
/**
* @brief Function implementing the KEY_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKEY_Task */
void StartKEY_Task(void *argument)
{
  /* USER CODE BEGIN StartKEY_Task */
  /* Infinite loop */
 // 消抖状态机
  enum { KS_IDLE, KS_PRESS } key1_state = KS_IDLE, key2_state = KS_IDLE;
  uint32_t key1_timer = 0, key2_timer = 0;
  const uint32_t DEBOUNCE_MS = 50;
  const uint32_t LONG_PRESS_MS = 1000;

  for(;;)
  {
    osDelay(10);  // 10ms 扫描周期

    // ────────── KEY1 扫描 ──────────
    GPIO_PinState k1 = HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN);
    switch (key1_state)
    {
      case KS_IDLE:
        if (k1 == GPIO_PIN_RESET) {
          key1_state = KS_PRESS;
          key1_timer = 0;
        }
        break;

      case KS_PRESS:
        key1_timer += 10;
        if (k1 == GPIO_PIN_SET) {          // 释放
          if (key1_timer >= DEBOUNCE_MS && key1_timer < LONG_PRESS_MS) {
            // 短按：锁定当前姿态
            float cur_roll  = euler.roll;
            float cur_pitch = euler.pitch;
            float cur_yaw   = euler.yaw - yaw_zero_offset;
            // 归一化到 -180~180
            if (cur_yaw > 180.0f)  cur_yaw -= 360.0f;
            if (cur_yaw < -180.0f) cur_yaw += 360.0f;
            Set_Target_Angles(cur_roll, cur_pitch, cur_yaw);
          }
          key1_state = KS_IDLE;
        }
        else if (key1_timer >= LONG_PRESS_MS) {
          key1_state = KS_IDLE;  // 长按暂不处理
        }
        break;
    }

    // ────────── KEY2 扫描 ──────────
    GPIO_PinState k2 = HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);
    switch (key2_state)
    {
      case KS_IDLE:
        if (k2 == GPIO_PIN_RESET) {
          key2_state = KS_PRESS;
          key2_timer = 0;
        }
        break;

      case KS_PRESS:
        key2_timer += 10;
        if (k2 == GPIO_PIN_SET) {
          if (key2_timer >= DEBOUNCE_MS && key2_timer < LONG_PRESS_MS) {
            // 短按：恢复默认角度（-6°, 0°, 0°）
            Set_Target_Angles(-6.0f, 0.0f, 0.0f);
          }
          key2_state = KS_IDLE;
        }
        else if (key2_timer >= LONG_PRESS_MS) {
          key2_state = KS_IDLE;
        }
        break;
    }
  }
  /* USER CODE END StartKEY_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

