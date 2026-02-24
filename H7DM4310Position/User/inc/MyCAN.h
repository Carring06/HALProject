#ifndef MYCAN_H_
#define MYCAN_H_

#include "stm32h7xx_hal.h"
#include "fdcan.h"

void MyFDCAN_Init(void);
void FDCAN1_FilterInit(void);
void MyFDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len);

#endif /* MYCAN_H_ */
