#include "vofa.h"
 
/**
 * 函数功能：将浮点数拆分成4个字节
 * 入口参数：Fdata：需要操作的浮点数
 * 入口参数：ArrayByte：数组地址
 * 返 回 值：无
 */
void Float_to_Byte(float Fdata, uint8_t *ArrayByte)
{
    Vofa_Type Vofa;
    Vofa.Fdata = Fdata;
    ArrayByte[0] = Vofa.Adata;       // 低字节（0-7位）
    ArrayByte[1] = Vofa.Adata >> 8;  // 8-15位
    ArrayByte[2] = Vofa.Adata >> 16; // 16-23位
    ArrayByte[3] = Vofa.Adata >> 24; // 高字节（24-31位）
}
 
/**
 * 函数功能：串口发送数据到VOFA+上位机
 * 入口参数：huart：串口句柄，比如 &huart1
 * 入口参数：data：要发送的浮点数数组
 * 入口参数：count：浮点数的个数
 * 返 回 值：无
 */
void VOFA_SendFloats(UART_HandleTypeDef *huart, float *data, uint8_t count)
{
    uint8_t Byte[4];
    uint8_t Tail[4] = {0x00, 0x00, 0x80, 0x7F};
 
    // 逐个发送浮点数
    for (uint8_t i = 0; i < count; i++)
    {
        Float_to_Byte(data[i], Byte);
        HAL_UART_Transmit(huart, Byte, 4, HAL_MAX_DELAY);
    }
 
    // 发送包尾
    HAL_UART_Transmit(huart, Tail, 4, HAL_MAX_DELAY);
}
 
/**
 * 示例：发送两个浮点数到VOFA+
 */
void JustFloat_Example(void)
{
    float a = 50.0f;
    float b = 60.0f;
    float send_data[2] = {a, b};
 
    // 通过USART1发送
    VOFA_SendFloats(&huart1, send_data, 2);
}
 
/**
 * 函数功能：发送目标速度和实际速度到 VOFA+ 显示曲线
 * 入口参数：huart：串口句柄（如 &huart1）
 * 入口参数：TargetSpeed：目标速度（物理值，如 4.0f）
 * 入口参数：ActualSpeed：实际速度（物理值，如 3.8f）
 * 返 回 值：无
 */
void VOFA_Show_Speed(UART_HandleTypeDef *huart, float TargetSpeed, float ActualSpeed, float Out)
{
    // 打包一个目标速度，一个实际速度，一个输出值到数组
    float send_data[3] = {TargetSpeed, ActualSpeed, Out};
 
    // 调用已有的 VOFA 发送函数
    VOFA_SendFloats(huart, send_data, 3);
}
 
/**
 * 函数功能：发送目标位置和实际位置到 VOFA+ 显示曲线
 * 入口参数：huart：串口句柄（如 &huart1）
 * 入口参数：TargetPos：目标位置（物理值）
 * 入口参数：ActualPos：实际位置（物理值）
 * 入口参数：Out：输出值
 * 返 回 值：无
 */
void VOFA_Show_Position(UART_HandleTypeDef *huart, float TargetPos, float ActualPos, float Out)
{
    // 打包三个值到数组
    float send_data[3] = {TargetPos, ActualPos, Out};
 
    // 调用已有的 VOFA 发送函数
    VOFA_SendFloats(huart, send_data, 3);
}
 
/**
 * 函数功能：发送目标位置、实际位置和速度输出到 VOFA+ 显示曲线
 * 入口参数：huart：串口句柄（如 &huart1）
 * 入口参数：TargetPos：目标位置（物理值）
 * 入口参数：ActualPos：实际位置（物理值）
 * 入口参数：SpeedOut：速度输出（物理值）
 * 返 回 值：无
 */
void VOFA_Show_Pos_SpeedOut(UART_HandleTypeDef *huart, float TargetPos, float ActualPos, float SpeedOut)
{
    // 打包三个值到数组
    float send_data[3] = {TargetPos, ActualPos, SpeedOut};
 
    // 调用已有的 VOFA 发送函数
    VOFA_SendFloats(huart, send_data, 3);
}
