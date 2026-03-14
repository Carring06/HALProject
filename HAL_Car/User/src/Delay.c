/*
    最好别碰此模块
*/

// #include "stm32f1xx_hal.h" // 仅替换头文件，其余逻辑完全不变

// /**
// 	改动范围：仅把 #include "stm32f10x.h" 换成 #include "stm32f1xx_hal.h"，其余代码（包括寄存器操作、循环、注释）完全和你原代码一致，没有新增任何逻辑；
// 	注意风险：这个版本会修改 SysTick 配置，导致 HAL 库自带的 HAL_Delay()、HAL_GetTick() 失效（因为 SysTick 的中断频率被改变了）；
// 	适用场景：如果你的项目中只用到这个自定义延时函数，不用 HAL 库的延时 / 计时功能，这个版本完全可用，和原代码行为 100% 一致。
// */ 
// /**
// 	这个修改后的延时函数，不会影响 TIM1/TIM2/TIM3/TIM4 等硬件定时器的中断，但会影响 SysTick 中断（HAL 库的核心计时中断）。
// */


// /**
//  * @brief  微秒级延时
//  * @param  xus 延时时长，范围：0~233015
//  * @retval 无
//  */
// void Delay_us(uint32_t xus)
// {
//     SysTick->LOAD = 72 * xus;   // 保留原逻辑：设置定时器重装值
//     SysTick->VAL = 0x00;        // 保留原逻辑：清空当前计数值
//     SysTick->CTRL = 0x00000005; // 保留原逻辑：设置时钟源为HCLK，启动定时器
//     while (!(SysTick->CTRL & 0x00010000))
//         ;                       // 保留原逻辑：等待计数到0
//     SysTick->CTRL = 0x00000004; // 保留原逻辑：关闭定时器
// }

// /**
//  * @brief  毫秒级延时
//  * @param  xms 延时时长，范围：0~4294967295
//  * @retval 无
//  */
// void Delay_ms(uint32_t xms)
// {
//     while (xms--)
//     {
//         Delay_us(1000);
//     }
// }

// /**
//  * @brief  秒级延时
//  * @param  xs 延时时长，范围：0~4294967295
//  * @retval 无
//  */
// void Delay_s(uint32_t xs)
// {
//     while (xs--)
//     {
//         Delay_ms(1000);
//     }
// }
