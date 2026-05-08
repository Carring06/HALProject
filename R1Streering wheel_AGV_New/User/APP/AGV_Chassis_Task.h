#ifndef __AGV_Chassis_Task_H
#define __AGV_Chassis_Task_H

#include "main.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
typedef float fp32;

//7545
//6101
//4782
//2036

//��е��λһ��Ҫ����0~4096�м��,�ǵ�
#define L_Q_6020_Middle_ECD 7545.0f   
#define L_H_6020_Middle_ECD 6101.0f
#define R_H_6020_Middle_ECD 4782.0f
#define R_Q_6020_Middle_ECD 2036.0f

#define CHASSIS_OFFSET_X  0.315f
#define CHASSIS_OFFSET_Y  0.295f
#define CHASSIS_OFFSET_L  0.27f



#define sin_l  0.74f		//CHASSIS_OFFSET_Y/CHASSIS_OFFSET_L
#define cos_l  0.69f		//CHASSIS_OFFSET_X/CHASSIS_OFFSET_L


#define Pi  3.1415f 
#define WHEEL_RADIUS  0.0625f		//轮子半径
#define WHEEL_PERIMETER   0.3926875f  	//轮子周长



//���ӵ�����ļ��ٱ�
#define CHASSIS_DECELE_RATIO 268.0f/17.0f 



//��̨����̨����Ĵ�����
#define GIMBAL_TO_4310  0.5f



//ϵͳ����״̬
typedef enum
{
Initing=0,
Running=1,	
}SystemValue;

typedef enum
{
RC_NO_INIT=0,
CHASSIS_FORWARD = 1,
AGV_CHASSIS_FOLLOW_GIMBAL = 2,
GIMBAL_FORWARD = 3,
}Chassis_Mode;


//�����ٶȽṹ��
typedef struct chassis_speed
{
float vx;	//�����ٶ�
float vy;	//�����ٶ�
float vw;	//��ת�ٶ�(��ʱ��Ϊ��)	
}Chassis_Speed;

void AGV_Chassis_Task(void* argument );
void AGV_Chassis_Init(void);
void Wheel_Angle_Last_Init(void);
void Motor_pid_init(void);
void RemoteControlChassis(void);
void Absolute_Cal(Chassis_Speed* absolute_speed, fp32 angle);
fp32 Find_min_Angle(int16_t angle1,fp32 angle2);
void AGV_speed_calc(Chassis_Speed *speed, int16_t* out_speed) ;
void AGV_angle_calc(Chassis_Speed *speed, fp32* out_angle);
void AGV_Set_Motor_angle(fp32 *out_angle, DM_Motor_Ctrl_Typedef* Motor ) ;
void AGV_Set_Motor_Speed(int16_t*out_speed, DJI_Motor_Ctrl_Typedef* Motor ) ;
void CHASSIS_Single_Loop_Out(void);
void DM_Motor_Four_Ctrl(fp32 tor1, fp32 tor2, fp32 tor3, fp32 tor4);
void Chassis_DM_Motor_Enable(void);
#endif
