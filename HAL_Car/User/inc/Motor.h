#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f1xx_hal.h"
#include "tim.h"

void Motor_PWMStart(void);
void MotorM1_Set(int16_t Speed);
void MotorM2_Set(int16_t Speed);

void StopMotor(void);

#endif /* MOTOR_H_ */
