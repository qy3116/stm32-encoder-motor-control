/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_SAMPLE_PERIOD_MS 100U
#define APP_SPEED_STEP       10
#define APP_SPEED_MIN        0
#define APP_SPEED_MAX        100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static int16_t encoder_cnt = 0;
static uint16_t encoder_delta = 0;
static int16_t target_speed = 0;
static uint16_t motor_rpm = 0;
static uint32_t last_sample_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_HandleKey(void);
static void App_UpdateEncoder(void);
static void App_UpdateDisplay(void);
static int16_t App_ClampSpeed(int16_t speed);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Initialize all board-level modules used by the application loop. */
static void App_Init(void)
{
  Motor_Init();
  Encoder_Init();

  OLED_Init();
  OLED_Clear();

  Motor_SetSpeed(target_speed);
  last_sample_time = HAL_GetTick();

  App_UpdateDisplay();
}

/* Convert one key event into a bounded motor speed command. */
static void App_HandleKey(void)
{
  Key_Value_t key_value;
  int16_t new_speed;

  key_value = Key_Scan();
  new_speed = target_speed;

  if (key_value == KEY_ADD)
  {
    new_speed += APP_SPEED_STEP;
  }
  else if (key_value == KEY_SUB)
  {
    new_speed -= APP_SPEED_STEP;
  }
  else
  {
    return;
  }

  new_speed = App_ClampSpeed(new_speed);

  if (new_speed != target_speed)
  {
    target_speed = new_speed;
    Motor_SetSpeed(target_speed);
  }
}

/* Sample the encoder once per application period and derive RPM from delta. */
static void App_UpdateEncoder(void)
{
  encoder_cnt = Encoder_GetCount();
  encoder_delta = Encoder_GetAbsDelta();
  motor_rpm = Encoder_CalcRPM(encoder_delta, APP_SAMPLE_PERIOD_MS);
}

/* Refresh only the text fields used on the first four OLED pages. */
static void App_UpdateDisplay(void)
{
  OLED_ShowString(0, 0, "Speed:");
  OLED_ShowNum(42, 0, (uint32_t)target_speed, 3);

  OLED_ShowString(0, 1, "Cnt:");
  OLED_ShowSignedNum(42, 1, encoder_cnt, 5);

  OLED_ShowString(0, 2, "Del:");
  OLED_ShowNum(30, 2, encoder_delta, 5);

  OLED_ShowString(0, 3, "RPM:");
  OLED_ShowNum(30, 3, motor_rpm, 5);
}

/* Keep target speed inside the range accepted by Motor_SetSpeed(). */
static int16_t App_ClampSpeed(int16_t speed)
{
  if (speed > APP_SPEED_MAX)
  {
    return APP_SPEED_MAX;
  }

  if (speed < APP_SPEED_MIN)
  {
    return APP_SPEED_MIN;
  }

  return speed;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  App_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
     * 按键改变target_speed
     */
    App_HandleKey();
    /*
     * HAL_GetTick() 返回系统运行时间，单位 ms。
     * 例如系统启动后运行了 12345ms，则 HAL_GetTick() = 12345。
     */
    if ((uint32_t)(HAL_GetTick() - last_sample_time) >= APP_SAMPLE_PERIOD_MS)
    {
      /*
       * 更新时间戳。
       * 这表示这一次转速计算已经完成。
       */
      last_sample_time += APP_SAMPLE_PERIOD_MS;

      /*
       * 每 100ms 调用一次，所以传入 sample_period_ms。
       */
      App_UpdateEncoder();

      /*
      * OLED显示。
      * 注意：OLED显示放在这里，表示每100ms刷新一次。
      */
      App_UpdateDisplay();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
