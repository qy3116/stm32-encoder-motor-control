/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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

#include "app.h"

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

/* 应用层状态放在 User/app/app.c 中，这里只负责创建 FreeRTOS 任务。 */

/* USER CODE END Variables */
/* EncoderTask 的任务句柄和创建参数。 */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* OLEDTask 的任务句柄和创建参数。 */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* KeyTask 的任务句柄和创建参数。 */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* MotorTask 的任务句柄和创建参数。 */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* VofaTask 的任务句柄和创建参数，用来周期发送 PID 串口曲线数据。 */
osThreadId_t VofaTaskHandle;
const osThreadAttr_t VofaTask_attributes = {
  .name = "VofaTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* LEDTask 的任务句柄和创建参数，用来观察调度器是否在运行。 */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartEncoderTask(void *argument);
void StartOLEDTask(void *argument);
void StartKeyTask(void *argument);
void StartMotorTask(void *argument);
void StartVofaTask(void *argument);
void StartLEDTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /*
   * 在任何任务启动前先复位应用层共享状态。
   * 各外设的硬件初始化放到拥有该外设的任务里完成。
   */
  App_Init();

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
  /* 创建编码器采样任务。 */
  EncoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &EncoderTask_attributes);

  /* 创建 OLED 显示任务。 */
  OLEDTaskHandle = osThreadNew(StartOLEDTask, NULL, &OLEDTask_attributes);

  /* 创建按键扫描任务。 */
  KeyTaskHandle = osThreadNew(StartKeyTask, NULL, &KeyTask_attributes);

  /* 创建电机输出任务。 */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* 创建 VOFA 串口调试任务。 */
  VofaTaskHandle = osThreadNew(StartVofaTask, NULL, &VofaTask_attributes);

  /* 创建 LED 心跳任务。 */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /*
   * 如果 heap 不够，osThreadNew() 会返回 NULL。
   * 这里主动检查，避免出现“部分任务没创建成功，但程序表面还在跑”的假象。
   */
  if ((EncoderTaskHandle == NULL) ||
      (OLEDTaskHandle == NULL) ||
      (KeyTaskHandle == NULL) ||
      (MotorTaskHandle == NULL) ||
      (VofaTaskHandle == NULL) ||
      (LEDTaskHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief EncoderTask 线程入口。
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderTask */
  App_EncoderTask();
  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief OLEDTask 线程入口。
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void *argument)
{
  /* USER CODE BEGIN StartOLEDTask */
  App_OLEDTask();
  /* USER CODE END StartOLEDTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief KeyTask 线程入口。
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
  App_KeyTask();
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief MotorTask 线程入口。
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  App_MotorTask();
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartVofaTask */
/**
* @brief VofaTask 线程入口。
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartVofaTask */
void StartVofaTask(void *argument)
{
  /* USER CODE BEGIN StartVofaTask */
  App_VofaTask();
  /* USER CODE END StartVofaTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief LEDTask 线程入口。
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* 无限循环：LEDTask 周期性翻转 PA8，方便确认调度器仍在运行。 */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(500);
  }
  /* USER CODE END StartLEDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

