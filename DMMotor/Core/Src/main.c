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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MyCAN.h"
#include "DM4310.h"
#include "vofa.h"
#include "Transform.h"
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

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
PID_SpeedStruct Speed_PID;
float Target_Speed = 0.0;
uint8_t count = 0;
uint8_t debounceCounterPB1 = 0;         // 定义PB1消抖计数器
uint8_t lastPinStatePB1 = GPIO_PIN_SET; // 定义PB1上次按键状态

uint8_t debounceCounterPB11 = 0;         // 定义PB11消抖计数器
uint8_t lastPinStatePB11 = GPIO_PIN_SET; // 定义PB11上次按键状态

// CAN_HandleTypeDef hcan;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void checkButtons(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void checkButtons(void)
{
  uint8_t currentPinStatePB1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
  uint8_t currentPinStatePB11 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

  // 处理PB1按键
  if (currentPinStatePB1 == GPIO_PIN_RESET)
  {
    if (lastPinStatePB1 == GPIO_PIN_RESET)
    {
      debounceCounterPB1++;
      if (debounceCounterPB1 >= 10) // 使用固定的阈值10
      {
        // 当按键状态为低电平的时间超过阈值，认为按键被稳定按下
        Target_Speed += 10;
        debounceCounterPB1 = 0; // 按键处理后重置计数器
      }
    }
    else
    {
      debounceCounterPB1 = 1; // 如果上次是高电平，这次是低电平，计数器设为1
    }
  }
  else
  {
    debounceCounterPB1 = 0; // 如果按键状态不是低电平，重置计数器
  }

  // 处理PB11按键
  if (currentPinStatePB11 == GPIO_PIN_RESET)
  {
    if (lastPinStatePB11 == GPIO_PIN_RESET)
    {
      debounceCounterPB11++;
      if (debounceCounterPB11 >= 10) // 使用固定的阈值10
      {
        // 当按键状态为低电平的时间超过阈值，认为按键被稳定按下
        Target_Speed -= 10;
        debounceCounterPB11 = 0; // 按键处理后重置计数器
      }
    }
    else
    {
      debounceCounterPB11 = 1; // 如果上次是高电平，这次是低电平，计数器设为1
    }
  }
  else
  {
    debounceCounterPB11 = 0; // 如果按键状态不是低电平，重置计数器
  }

  lastPinStatePB1 = currentPinStatePB1;   // 更新PB1上次按键状态
  lastPinStatePB11 = currentPinStatePB11; // 更新PB11上次按键状态
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  Speed_PID.Kp = 0.054982;
  Speed_PID.Ki = 0.003175;
  Speed_PID.Kd = 0.000001275;
  Speed_PID.I_Max = 5.00;
  Speed_PID.Out_Max = 1.00;
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MyCAN_Init();
  Enable_DM4310(); // 使能DM4310
  // Disable_DM4310();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    checkButtons();

    if (Target_Speed > 20)
    {
      Target_Speed = -0;
    }
    else if (Target_Speed < -20)
    {
      Target_Speed = -0;
    }

    HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    //    uint16_t Cur_torque = ((RxData[4] & 0x0F) << 8) | RxData[5];
    uint16_t Cur_Speed = RxData[3] << 4 | (RxData[4] & 0xF0) >> 4;
    float ActualSpeed = uint_to_float(Cur_Speed, -30, 30, 12);
    Set_DM4310_Speed(Target_Speed, ActualSpeed, &Speed_PID);

    HAL_Delay(10);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
