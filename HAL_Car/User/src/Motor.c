#include "Header.h"

#define GPIO_AIN GPIOB
#define AIN1 GPIO_PIN_12
#define AIN2 GPIO_PIN_13
#define GPIO_BIN GPIOB
#define BIN1 GPIO_PIN_14
#define BIN2 GPIO_PIN_15

void Motor_PWMStart(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

/**
 * Speed: -1000~1000
 * ARR已经设为：1000 - 1
 */
void MotorM1_Set(int16_t Speed)
{
    if (Speed < 0)
    {
        HAL_GPIO_WritePin(GPIO_AIN, AIN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_AIN, AIN2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -Speed);
    }
    else if (Speed >= 0)
    {
        HAL_GPIO_WritePin(GPIO_AIN, AIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_AIN, AIN2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, +Speed);
    }
    //    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					//在调用这个函数前，启动一次就可以了
}
/**
 * Speed: -1000~1000
 * ARR已经设为：1000 - 1
 */
void MotorM2_Set(int16_t Speed)
{
    if (Speed < 0)
    {
        HAL_GPIO_WritePin(GPIO_BIN, BIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_BIN, BIN2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -Speed);
    }
    else if (Speed >= 0)
    {
        HAL_GPIO_WritePin(GPIO_BIN, BIN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_BIN, BIN2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, +Speed);
    }
    //    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);						//在调用这个函数前，启动一次就可以了
}

void StopMotor(void)
{
    MotorM1_Set(0);
    MotorM2_Set(0);
}
