#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include "usart.h"

char Serial_RxPacket[100];
uint8_t Serial_RxFlag;
static uint8_t RxData;       // 存储中断接收到的单个字节
static uint8_t RxState = 0;  // 接收状态机（和原代码逻辑一致）
static uint8_t pRxPacket = 0;// 数据包指针（和原代码逻辑一致）

/**
 * @brief  串口初始化（适配HAL库，对应原Serial_Init）
 * @note   完全保留原功能：USART1 9600波特率，中断接收数据包
 */

void BLE_Serial_Init(void)
{
    // 不再手动初始化USART1和GPIO，改用CubeMX生成的MX_USART1_UART_Init()

    // 仅保留NVIC和中断开启（核心逻辑）
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1); // 抢占优先级1，子优先级1(可不用)
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // 开启USART1接收中断
    HAL_UART_Receive_IT(&huart1, &RxData, 1);
}

/*
void BLE_Serial_Init(void)
{
    // 1. 使能时钟（替代标准库RCC_APB2PeriphClockCmd）
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 2. GPIO初始化（替代标准库GPIO_Init）
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PA9 -> USART1_TX（复用推挽输出，和原代码一致）
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA10 -> USART1_RX（上拉输入，和原代码一致）
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. USART初始化（替代标准库USART_Init）
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    // 4. NVIC配置（替代标准库NVIC_Init）
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);  // 抢占优先级1，子优先级1（和原代码一致）
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // 5. 开启USART1接收中断（替代标准库USART_ITConfig）
    HAL_UART_Receive_IT(&huart1, &RxData, 1);
}
*/
    /**
     * @brief  发送单个字节（替代原Serial_SendByte）
     */
void BLE_Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);
}

/**
 * @brief  发送数组（替代原Serial_SendArray）
 */
void BLE_Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart1, Array, Length, HAL_MAX_DELAY);
}

/**
 * @brief  发送字符串（和原代码逻辑完全一致）
 */
void BLE_Serial_SendString(char *String)
{
    uint16_t i = 0;
    while (String[i] != '\0')
    {
        BLE_Serial_SendByte(String[i]);
        i ++;
    }
}

/**
 * @brief  幂运算（和原代码完全一致）
 */
uint32_t BLE_Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  发送数字（和原代码逻辑完全一致）
 */
void BLE_Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i ++)
    {
        BLE_Serial_SendByte(Number / BLE_Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  printf重定向（和原代码逻辑完全一致）
 */
int fputc(int ch, FILE *f)
{
    BLE_Serial_SendByte(ch);
    return ch;
}

/**
 * @brief  格式化发送（和原代码逻辑完全一致）
 */
void BLE_Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    BLE_Serial_SendString(String);
}

/**
 * @brief  USART1中断服务函数（HAL库标准写法）
 * @note   替代原USART1_IRQHandler，调用HAL库中断处理函数
 */
//void USART1_IRQHandler(void)
//{
//    HAL_UART_IRQHandler(&huart1);
//}

/**
 * @brief  UART接收完成回调函数（处理中断接收的字节）
 * @note   原代码的中断逻辑迁移到这里，完全保留原状态机
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (RxState == 0)
        {
            if (RxData == '[' && Serial_RxFlag == 0)
            {
                RxState = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)
        {
            if (RxData == ']')
            {
                RxState = 0;
                Serial_RxPacket[pRxPacket] = '\0';
                Serial_RxFlag = 1;
            }
            else
            {
                Serial_RxPacket[pRxPacket] = RxData;
                pRxPacket ++;
                // 防止数组越界（原代码未处理，这里补充）
                if (pRxPacket >= 99) pRxPacket = 99;
            }
        }

        // 重新开启接收中断，等待下一个字节
        HAL_UART_Receive_IT(&huart1, &RxData, 1);
    }
}
