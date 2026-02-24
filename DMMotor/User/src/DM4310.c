#include "DM4310.h"


#define motor_id 0x01
#define mode_id 0x00                   // MIT模式的ID = 0x000 + CAN_ID

void Enable_DM4310(void)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;       //两个宏定义根据CAN_ID和Mode改

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    MyCAN_SendData(id, data, 8); 
}

void Disable_DM4310(void)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;       //两个宏定义根据CAN_ID和Mode改

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;
    MyCAN_SendData(id, data, 8); 
}

/*
额定扭矩 3.5NM
峰值扭矩 11NM    0.8过流
        12.5NM  0.98过流
*/

/*不加PID测试版*/
// void Set_DM4310_Torque(uint8_t torque)
// {
//     uint8_t data[8];
//     uint16_t id = motor_id + mode_id;       //两个宏定义根据CAN_ID和Mode改

//     data[0] = 0x00;
//     data[1] = 0x00;
//     data[2] = 0x00;
//     data[3] = 0x00;
//     data[4] = 0x00;
//     data[5] = 0x00;
//     data[6] = (torque >> 8) & 0x0F;
//     data[7] = torque & 0xFF;

//     MyCAN_SendData(id, data, 8);
// }

/*
    加PID测试版
    速度环——控速
*/
void Set_DM4310_Speed(float Des_Speed, float Cur_Speed, PID_SpeedStruct *PID_Speed)
{
    /*对输入速度进行限幅~达妙调试助手VMax = 30，但实测Max只有23*/
    if (Des_Speed > 23.0)
    {
        Des_Speed = 23.0;
    }
    else if (Des_Speed < -23.0)
    {
        Des_Speed = -23.0;
    }

    uint8_t data[8];
    uint16_t id = motor_id + mode_id; // 两个宏定义根据CAN_ID和Mode改

    //误差计算
    PID_Speed->Last_E = PID_Speed->E;
    PID_Speed->E = Des_Speed - Cur_Speed;
    //P
    PID_Speed->P_Out = PID_Speed->Kp * PID_Speed->E;
    //I
    PID_Speed->I_Out += PID_Speed->Ki * PID_Speed->E;
    if (PID_Speed->I_Out > PID_Speed->I_Max)
    {
        PID_Speed->I_Out = PID_Speed->I_Max;
    }
    else if (PID_Speed->I_Out < -PID_Speed->I_Max)
    {
        PID_Speed->I_Out = -PID_Speed->I_Max;
		}
    //D
    PID_Speed->D_Out = PID_Speed->Kd * (PID_Speed->E - PID_Speed->Last_E);
    
		
    //总输出
    PID_Speed->Total_Out = PID_Speed->P_Out + PID_Speed->I_Out + PID_Speed->D_Out;
    if(PID_Speed->Total_Out > PID_Speed->Out_Max)
    {
        PID_Speed->Total_Out = PID_Speed->Out_Max;
    }
    else if(PID_Speed->Total_Out < -PID_Speed->Out_Max)
    {
        PID_Speed->Total_Out = -PID_Speed->Out_Max;
    }

    VOFA_Show_Speed(&huart1, Des_Speed, Cur_Speed, PID_Speed->Total_Out);

    uint16_t ActualSpeed = float_to_uint(PID_Speed->Total_Out, -23, 23, 12);

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = (ActualSpeed >> 8) & 0x0F;
    data[7] = ActualSpeed & 0xFF;

    MyCAN_SendData(id, data, 8);


}
