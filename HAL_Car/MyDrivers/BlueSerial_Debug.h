#ifndef BlueSerial_Debug_h
#define BlueSerial_Debug_h

#include "stm32f1xx_hal.h"
#include "string.h"
#include <stdlib.h>
#include "BLE_Serial.h"
#include "Motor_driver.h"

void BlueSerial_DebugSpeedPID(void);

void BlueSerial_DebugAnglePID(void);

void BlueSerial_DebugPID(void);

#endif /* BlueSerial_Debug_h */
