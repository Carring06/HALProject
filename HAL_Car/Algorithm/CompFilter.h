#ifndef COMPFILTER_H
#define COMPFILTER_H

#include "stm32f1xx_hal.h"
#include "MPU6050.h"
#include <math.h>

extern float AngleAcc;
extern float AngleGyro;
extern float Angle;

void Complementary_Filter(void);
float GetPitchAngle(void);

#endif
