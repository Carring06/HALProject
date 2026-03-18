#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "stm32f1xx_hal.h" // Device header
#include "Motor.h"
#include "CompFilter.h"
#include "PID.h"

extern PID_t Car_SpeedM1PID;
extern PID_t Car_SpeedM2PID;
extern PID_t Car_UprightM1PID;
extern PID_t Car_UprightM2PID;

void Car_SpeedM1Driver(void);

void Car_SpeedM2Driver(void);

// void Car_UprightM1Driver(float ComeInOut);

// void Car_UprightM2Driver(float ComeInOut);

// void Car_SpeedM1Driver(void);

// void Car_SpeedM2Driver(void);

void Car_UprightM1Driver(void);

void Car_UprightM2Driver(void);

#endif
