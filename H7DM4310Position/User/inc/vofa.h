#ifndef __VOFA_H
#define __VOFA_H

#include "stm32h7xx_hal.h"
#include "usart.h"
#include "string.h"

// void VOFA_ShowData(float Target, float Current);

typedef union
{
    float Fdata;    // 以浮点数形式读取变量
    uint32_t Adata; // 以32位无符号形式读取变量
} Vofa_Type;

void Float_to_Byte(float Fdata, uint8_t *ArrayByte);
void VOFA_SendFloats(UART_HandleTypeDef *huart, float *data, uint8_t count);
void JustFloat_Example(void);
void VOFA_Show_Speed(UART_HandleTypeDef *huart, float TargetSpeed, float ActualSpeed, float Out);
void VOFA_Show_Position(UART_HandleTypeDef *huart, float TargetPos, float ActualPos, float Out);
void VOFA_Show_Pos_SpeedOut(UART_HandleTypeDef *huart, float TargetPos, float ActualPos, float SpeedOut);

#endif
