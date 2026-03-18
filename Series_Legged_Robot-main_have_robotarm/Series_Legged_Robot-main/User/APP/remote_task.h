#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H

#include "main.h"
#include "chassisR_task.h"
#include "ins_task.h"
//#include "uart_bsp.h"
#include "usart.h"
//#include "Remote_Control.h"
void Remote_task(void);

void remote_crtl(chassis_t *chassis,float dt);
#endif
