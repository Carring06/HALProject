#include "Servo.h"

void Servo_Init(void)
{
    // 启动TIM3 CH1的PWM，舵机PWM只需启动一次，无需重复启动
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void Servo_SetAngle(float angle)
{
	//Set the angle of the servo motor
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, angle / 180 * 2000 + 500);

}
