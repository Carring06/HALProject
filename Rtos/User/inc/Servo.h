#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f1xx.h"  
#include "tim.h"

void Servo_SetAngle(float angle);
void Servo_Init(void);

#endif /* SERVO_H_ */
