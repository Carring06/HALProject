#ifndef __AGV_Chassis_Task_H
#define __AGV_Chassis_Task_H

#include "main.h"
#include <stdbool.h>//c
#include "FreeRTOS.h"// FreeRTOS
#include "task.h"
#include "cmsis_os.h"
#include "DJI_Motor.h"//motor drive
#include "DM_Motor.h"
#include "pid.h"//pid
#include "Remote_Control.h"//REMOTE_CONTROL 
#include "fdcan.h"

typedef float fp32;

//7545
//6101
//4782
//2036

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



#define CHASSIS_DECELE_RATIO 268.0f/17.0f 



#define GIMBAL_TO_4310  0.5f



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



typedef struct chassis_speed
{
		float vx;  // 车体系X轴速度（前后方向，向前为正）
    float vy;  // 车体系Y轴速度（左右方向，向左为正）
    float vw;  // 旋转角速度（逆时针为正）
}Chassis_Speed;

void AGV_Chassis_Task(void);
void AGV_Chassis_Init(void);
void Wheel_Angle_Last_Init(void);
void Motor_pid_init(void);
void RemoteControlChassis(void);
void Absolute_Cal(Chassis_Speed *absolute_speed, fp32 angle);
fp32 Find_min_Angle(int16_t angle1, fp32 angle2);
void AGV_speed_calc(Chassis_Speed *speed, int16_t *out_speed);
void AGV_angle_calc(Chassis_Speed *speed, fp32 *out_angle);
void AGV_Set_Motor_Angle(fp32 *out_angle, DM_Motor_Ctrl_Typedef *Motor);
void AGV_Set_Motor_Speed(int16_t *out_speed, DJI_Motor_Ctrl_Typedef *Motor);
void CHASSIS_Single_Loop_Out(void);
void DM_Motor_Four_Ctrl(fp32 tor1, fp32 tor2, fp32 tor3, fp32 tor4);
void Chassis_DM_Motor_Enable(void);
void loop_f(float *angle, float max);
void AGV_chassis_Zero_Check(float Tar_Angle, float *Acl_Angle, float max);
#endif
