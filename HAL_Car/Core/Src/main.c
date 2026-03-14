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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*-----------------------------------------------------------------User's Code*/
#include "Header.h"
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
int16_t AX, AY, AZ, GX, GY, GZ;
uint16_t Num1;
uint16_t Num2;
int16_t speed = 0;
float Act_speedM1;
float Act_speedM2;
extern float target_speed;
uint16_t count;
uint16_t M1Encoder_Count;
uint16_t M2Encoder_Count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    /*---------------------------------------------------------------------按键扫描*/

    Key_Tick(); // 例如：1ms调用一次按键扫描

    /*-------------------------------------------------------------------编码器测速*/

    count++;
    if (count == 50)
    {


      count = 0;
    }

    /*---------------------------------------------------------------------互补滤波*/
//    static uint8_t count0 = 0;
//    count0++;
//    if (count0 == 10)
//    {
//      count0 = 0;
//    }
			

      /*----------------------------------------------------------------------PID计算*/
      static uint8_t count1 = 0;
      count1++;
      if (count1 == 50)
      {
        count1 = 0;
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        Complementary_Filter();
        Act_speedM1 = Encoder_GetM1() / 44.0 / 0.05 / 9.27666;  // 轮速计测量
        Act_speedM2 = -Encoder_GetM2() / 44.0 / 0.05 / 9.27666; // 轮速计测量
        Car_SpeedM1Driver();

        Car_SpeedM2Driver();
        // 1ms计算一次太快了，慢一点好吧
        //  Car_UprightM1Driver();

        // Car_UprightM2Driver();
      }
  }
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);                  // 启动TIM1中断
  HAL_TIM_Base_Start(&htim2);                     // 启动TIM2基本计时器
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动TIM3编码器
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动TIM4编码器
  OLED_GPIO_ForceInit();
  OLED_Init();
  BLE_Serial_Init();
  MPU6050_Init();
  Motor_PWMStart();
  Encoder_Init();
  BLE_Serial_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /*
      目前由于MPU6050_GetData()函数的代码效率未优化，效率低，因此暂时将此函数置于while循环中，勿在中断中调用，勿在其他函数中使用
    */
    // MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);——————已完善！！！
    /*
      放在中断，会卡死程序，猜测是在1ms中断执行该函数时，时间超过1ms导致又发生一次定时中断，导致卡死。解决方法（个人猜测）：①在进中断时，先清除中断标志位(HAL库自主完成了)②优化MOU6050读取速率(连续读取，减少延时)————————————
      ————————————————————yes
      豆包有话说：
                    准确识别了 “中断里执行耗时函数导致卡死” 的现象
                    理解了中断重入 / 嵌套的风险
                    给出了 “移出中断” 的正确方向
                    可以优化的点：
                    “先清中断标志位” 是治标，移出中断上下文才是治本
                    可以进一步优化 MPU6050 读取效率，让系统更流畅
    */

    ///////////////////////////做一些安全保护措施////////////////////////////////////////////
     Motor_Protection();
    // 改一下电机PWM配置
    //写个模块text模块
    //不烧录并复位是为了防止程序一下载电机就转造成破坏
    OLED_ShowPIDOut();

//    BlueSerial_DebugSpeedPID();
    BlueSerial_DebugPID();

        BLE_Serial_Printf("[plot,%f,%f,%f]", Car_SpeedM1PID.Target, Car_SpeedM1PID.Actual, Car_SpeedM2PID.Actual);
    // BLE_Serial_Printf("[plot,%f,%f,%f]", Car_UprightM1PID.Target, Car_UprightM1PID.Actual, Car_UprightM2PID.Actual);

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
