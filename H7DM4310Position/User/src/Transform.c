#include "Transform.h"

/*
    读数场景：用 uint_to_float 把「硬件/通信的无符号整数值」还原为「有物理意义的浮点数」→ 用于PID计算/数据显示（如编码值→实际速度、ADC值→电压）。
    写数场景：用 float_to_uint 把「有物理意义的浮点数」映射为「硬件/通信的无符号整数值」→ 用于控制输出（如PID输出→PWM占空比、目标速度→串口传输值）。
*/

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
