#include "stm32f1xx_hal.h"  // 仅替换头文件为HAL库文件
#include "Key.h"

// 保护按键定义，防止重复定义
#define KEY_PRESSED				1
#define KEY_UNPRESSED			0

#define KEY_TIME_DOUBLE			200
#define KEY_TIME_LONG			2000
#define KEY_TIME_REPEAT			100

#define GPIO_KEY1 				GPIO_PIN_1			/*By GPIOB*/
#define GPIO_KEY2				GPIO_PIN_0			/*By GPIOB*/
#define GPIO_KEY3				GPIO_PIN_5			/*By GPIOA*/
#define GPIO_KEY4				GPIO_PIN_4			/*By GPIOA*/

uint8_t Key_Flag[KEY_COUNT];

/**
 * @brief  按键初始化（HAL库版本）
 * @note   GPIO初始化由CubeMX自动生成，此处仅做函数占位
 * @note   CubeMX配置引脚方法
 */
void Key_Init(void)
{
    // 【CubeMX自动生成的代码，用户无需手动编写】
    // 1. GPIOB时钟使能
    // 2. 引脚初始化：PB1/P0/PA5/PA4为上拉输入
    // 此处仅保留函数，原函数的调用请自行补全
}

/**
 * @brief  获取按键当前状态（适配HAL库读取）
 * @param  n: 按键序号（KEY_1~KEY_4） 
 * @retval KEY_PRESSED/KEY_UNPRESSED
 */
uint8_t Key_GetState(uint8_t n)
{
    if (n == KEY_1)
    {
        // PB1为上拉输入，按下时为低电平（GPIO_PIN_RESET）
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_KEY1) == GPIO_PIN_RESET)
        {
            return KEY_PRESSED;
        }
    }
    else if (n == KEY_2)
    {
        // PB0为上拉输入，按下时为低电平
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_KEY2) == GPIO_PIN_RESET)
        {
            return KEY_PRESSED;
        }
    }
    else if (n == KEY_3)
    {
        // PA5为上拉输入，按下时为低电平
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_KEY3) == GPIO_PIN_RESET)
        {
            return KEY_PRESSED;
        }
    }
    else if (n == KEY_4)
    {
        // PA4为上拉输入，按下时为低电平
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_KEY4) == GPIO_PIN_RESET)
        {
            return KEY_PRESSED;
        }
    }
    return KEY_UNPRESSED;
}

// -------------------------- 按键处理核心部分，以下代码完全保留 --------------------------
uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
	if (Key_Flag[n] & Flag)
	{
		if (Flag != KEY_HOLD)
		{
			Key_Flag[n] &= ~Flag;
		}
		return 1;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count, i;
	static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
	static uint8_t S[KEY_COUNT];
	static uint16_t Time[KEY_COUNT];
	
	for (i = 0; i < KEY_COUNT; i ++)
	{
		if (Time[i] > 0)
		{
			Time[i] --;
		}
	}
	
	Count ++;
	if (Count >= 20)
	{
		Count = 0;
		
		for (i = 0; i < KEY_COUNT; i ++)
		{
			PrevState[i] = CurrState[i];
			CurrState[i] = Key_GetState(i);
			
			if (CurrState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_HOLD;
			}
			else
			{
				Key_Flag[i] &= ~KEY_HOLD;
			}
			
			if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
			{
				Key_Flag[i] |= KEY_DOWN;
			}
			
			if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_UP;
			}
			
			if (S[i] == 0)
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Time[i] = KEY_TIME_LONG;
					S[i] = 1;
				}
			}
			else if (S[i] == 1)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					Time[i] = KEY_TIME_DOUBLE;
					S[i] = 2;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_TIME_REPEAT;
					Key_Flag[i] |= KEY_LONG;
					S[i] = 4;
				}
			}
			else if (S[i] == 2)
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Key_Flag[i] |= KEY_DOUBLE;
					S[i] = 3;
				}
				else if (Time[i] == 0)
				{
					Key_Flag[i] |= KEY_SINGLE;
					S[i] = 0;
				}
			}
			else if (S[i] == 3)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
			}
			else if (S[i] == 4)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_TIME_REPEAT;
					Key_Flag[i] |= KEY_REPEAT;
					S[i] = 4;
				}
			}
		}
	}
}
