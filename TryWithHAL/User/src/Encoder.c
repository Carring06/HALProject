#include "stm32f1xx_hal.h"
#include "tim.h"

/**
 * @brief  编码器初始化（仅保留启动逻辑，所有底层配置由CubeMX生成）
 * @note   你只需要在CubeMX里配置好编码器，这里直接启动即可
 */
void Encoder_Init(void)
{
    // 仅启动编码器定时器（CubeMX已经完成所有底层配置，包括时钟、GPIO、编码器模式）
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

/**
 * @brief  获取编码器计数值（完全保留你的原逻辑，仅替换为HAL库接口）
 * @retval 编码器增量值（读取后清零）
 */
int16_t Encoder_Get(void)
{
    int16_t Temp;
    Temp = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return Temp;
}
