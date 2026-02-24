#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include "stm32f1xx_hal.h"
#include "math.h"

int float_to_uint(float x_float, float x_min, float x_max, int bits);

float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif /* TRANSFORM_H_ */
