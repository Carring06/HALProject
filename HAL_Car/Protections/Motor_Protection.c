#include "Header.h"

#define Blance_UpperLimit  60.0
#define Blance_LowerLimit -60.0
/*
    自测得的角度范围(不一定精准)：
    在俯仰角-5°到5°之间，平衡车有能力保持直立平衡，而在-5°到5°之外应当加以保护，以防止发生失控的情况。
*/
void Motor_Protection(void)
{
    // 读取俯仰角
    float pitch = GetPitchAngle();

    // 保护前提
    while (pitch < Blance_LowerLimit || pitch > Blance_UpperLimit)
    {
        // 停止电机转动
        StopMotor();
        if(Key_Check(KEY_3, KEY_SINGLE))
        {
            PID_Init(&Car_UprightM1PID);
            PID_Init(&Car_UprightM2PID);
            break;
        }
    }
    
    if (Key_Check(KEY_1, KEY_SINGLE))
    {
        PID_Init(&Car_UprightM1PID);
        PID_Init(&Car_UprightM2PID);
        /* 有待优化... */
        while (Key_Check(KEY_2, KEY_SINGLE) == 0)
        {
            StopMotor();
            PID_Init(&Car_UprightM1PID);
            PID_Init(&Car_UprightM2PID);
            // MotorM1_Set(0);
            // MotorM2_Set(0);
        }    
    }
}

