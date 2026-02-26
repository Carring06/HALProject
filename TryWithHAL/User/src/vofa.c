#include "vofa.h"
 
/*
	串口波特率： 115200
*/
 
 
/**
 * 函数功能：将浮点数拆分成4个字节
 * 输入参数：Fdata：需要操作的浮点数
 * 输入参数：ArrayByte：数组地址
 * 返回值：无
 */
void Float_to_Byte(float Fdata, uint8_t *ArrayByte)
{
    Vofa_Type Vofa;
    Vofa.Fdata = Fdata;
    ArrayByte[0] = Vofa.Adata;       // 低字节(0-7位)
    ArrayByte[1] = Vofa.Adata >> 8;  // 8-15位
    ArrayByte[2] = Vofa.Adata >> 16; // 16-23位
    ArrayByte[3] = Vofa.Adata >> 24; // 高字节(24-31位)
}
 
/**
 * 函数功能：串口发送数据到VOFA+上位机
 * 输入参数：huart：串口句柄，例如 &huart1
 * 输入参数：data：要发送的浮点数数组
 * 输入参数：count：浮点数的个数
 * 返回值：无
 */
void VOFA_SendFloats(UART_HandleTypeDef *huart, float *data, uint8_t count)
{
    // static uint8_t Byte[4];
    static uint8_t Tail[4] = {0x00, 0x00, 0x80, 0x7F};
    static uint8_t send_buffer[256];

    uint16_t total_len = 0;
    for (uint8_t i = 0; i < count; i++)
    {
        Float_to_Byte(data[i], &send_buffer[total_len]);
        total_len += 4;
    }
    memcpy(&send_buffer[total_len], Tail, 4);
    total_len += 4;

    HAL_UART_Transmit_DMA(huart, send_buffer, total_len);
}
 
/**
 * 示例：发送两个浮点数到VOFA+
 */
void JustFloat_Example(void)
{
    static float a = 0.0f;
    a ++;
    static float b = 100.0f;
    b --;
    if(a > 100.0f)
    {
        a = 0.0f;
    }
    if(b < 0.0f)
    {
        b = 100.0f;
    }
    float send_data[2] = {a, b};
 
    // 通过USART1发送
    VOFA_SendFloats(&huart2, send_data, 2);
}
 
/**
 * 函数功能：发送目标速度和实际速度到VOFA+ 显示曲线
 * 输入参数：huart：串口句柄(如 &huart1)
 * 输入参数：TargetSpeed：目标速度（物理值，如 4.0f）
 * 输入参数：ActualSpeed：实际速度（物理值，如 3.8f）
 * 输入参数：Out：输出值
 * 返回值：无
 */
void VOFA_Show_Speed(UART_HandleTypeDef *huart, float TargetSpeed, float ActualSpeed, float Out)
{
    // 打包一个目标速度，一个实际速度，一个输出值到数组
    float send_data[3] = {TargetSpeed, ActualSpeed, Out};
 
    // 调用已有的 VOFA 发送函数
    VOFA_SendFloats(huart, send_data, 3);
}
 
/**
 * 函数功能：发送目标位置和实际位置到VOFA+ 显示曲线
 * 输入参数：huart：串口句柄(如 &huart1)
 * 输入参数：TargetPos：目标位置（物理值）
 * 输入参数：ActualPos：实际位置（物理值）
 * 输入参数：Out：输出值
 * 返回值：无
 */
void VOFA_Show_Position(UART_HandleTypeDef *huart, float TargetPos, float ActualPos, float Out)
{
    // 打包三个值到数组
    float send_data[3] = {TargetPos, ActualPos, Out};
 
    // 调用已有的 VOFA 发送函数
    VOFA_SendFloats(huart, send_data, 3);
}
 
/**
 * 函数功能：发送目标位置、实际位置和速度输出到VOFA+ 显示曲线
 * 输入参数：huart：串口句柄(如 &huart1)
 * 输入参数：TargetPos：目标位置（物理值）
 * 输入参数：ActualPos：实际位置（物理值）
 * 输入参数：SpeedOut：速度输出（物理值）
 * 返回值：无
 */
void VOFA_Show_Pos_SpeedOut(UART_HandleTypeDef *huart, float TargetPos, float ActualPos, float SpeedOut)
{
    // 打包三个值到数组
    float send_data[3] = {TargetPos, ActualPos, SpeedOut};
 
    // 调用已有的 VOFA 发送函数
    VOFA_SendFloats(huart, send_data, 3);
}
