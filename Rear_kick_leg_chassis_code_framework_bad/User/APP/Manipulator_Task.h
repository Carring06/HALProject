#ifndef MANIPULATOR_TASK_H
#define MANIPULATOR_TASK_H

#include "stm32h7xx_hal.h"
#include "DM_Motor.h"
#include "Remote_Control.h"
#include "fdcan.h"
#include "bsp_can.h"
#include "user_lib.h"

void ManipulatorTask(void);

#endif /* MANIPULATOR_TASK_H */
