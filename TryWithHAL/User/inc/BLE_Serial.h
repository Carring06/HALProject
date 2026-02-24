#ifndef __BLE_SERIAL_H
#define __BLE_SERIAL_H

#include "stm32f1xx_hal.h"
#include <stdio.h>

extern char Serial_RxPacket[];
extern uint8_t Serial_RxFlag;

void BLE_Serial_Init(void);
void BLE_Serial_SendByte(uint8_t Byte);
void BLE_Serial_SendArray(uint8_t *Array, uint16_t Length);
void BLE_Serial_SendString(char *String);
void BLE_Serial_SendString(char *String);
void BLE_Serial_SendNumber(uint32_t Number, uint8_t Length);
void BLE_Serial_Printf(char *format, ...);

#endif
