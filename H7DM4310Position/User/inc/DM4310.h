#ifndef __DM4310_H__
#define __DM4310_H__

#include "stm32h7xx_hal.h"
#include "Transform.h"
#include "MyCAN.h"
#include "usart.h"
#include "vofa.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    int16_t E;
    float P_Out;
    float I_Out;
    float D_Out;
    float Total_Out;
    float I_Max;
    float Out_Max;
    uint16_t Last_E;
} PID_SpeedStruct,PID_PositionStruct;

void Enable_DM4310(void);
void Disable_DM4310(void);
void Set_DM4310_Speed(float Des_torque, float Cur_torque, PID_SpeedStruct *PID_Torque);
void Set_DM4310_Position(float Des_Pos, float Cur_Pos, float Cur_Speed,
                         PID_PositionStruct *PID_Pos, PID_SpeedStruct *PID_Speed);
void Set_DM4310_Torque(uint8_t torque);

#endif /* __DM4310_H__ */
