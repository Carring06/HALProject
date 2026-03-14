#ifndef HEADER_H_
#define HEADER_H_

#include "stm32f1xx_hal.h"
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
#include "string.h"
#include <math.h>
#include <stdlib.h>
#include "BLE_Serial.h"
#include "Encoder.h"
#include "key.h"
#include "LED.h"
#include "Motor.h"
#include "MPU6050.h"
#include "NRF24L01.h"
#include "OLED.h"
#include "vofa.h"
#include "Motor_driver.h"
#include "BlueSerial_Debug.h"
#include "ShowInOLED.h"
#include "Motor_Protection.h"
#include "MyI2C.h"
#include "Delay.h"
/*----------------------------------------------------------------PID Calculate*/
#include "PID.h"
/*---------------------------------------------------------Complementary Filter*/
#include "CompFilter.h"
/* USER CODE END Includes */

#endif /* HEADER_H_ */
