#ifndef MYCAN_H_
#define MYCAN_H_

#include "stm32f1xx_hal.h"

void MyCAN_Init(void);
void CAN_FilterInit(void);
void MyCAN_SendData(uint32_t id, uint8_t *data, uint8_t len);

#endif /* MYCAN_H_ */
