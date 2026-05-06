#ifndef __AGV_CHASSIS_TASK_H__
#define __AGV_CHASSIS_TASK_H__

#include "main.h"
#include "freertos.h"
#include "task.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "Remote_Control.h"
#include "bsp_uart.h"
#include "ins_task.h"
#include "bsp_can.h"
#include "user_lib.h"

/*
    各数值均需要实测
*/

// 底盘速度结构体
typedef struct
{
    float vx; // 底盘速度x轴 m/s
    float vy; // 底盘速度y轴 m/s
    float vw; // 底盘角速度 rad/s
} AGV_chassis_speed_Typedef;

// 云台控制结构体
typedef struct
{
    float vyaw;   // 云台yaw rad/s
    float vpitch; // 云台pitch rad/s
} AGV_gimbal_ctrl_Typedef;

// 系统运行状态
typedef enum {
    Initing = 0,
    Running = 1,
} SystemValue;

// 底盘电机ID枚举
typedef enum {
    CAN_CHASSIS_3508_ID    = 0x200,
    CAN_CHASSIS_LQ_3508_ID = 0x201,
    CAN_CHASSIS_LH_3508_ID = 0x202,
    CAN_CHASSIS_RH_3508_ID = 0x203,
    CAN_CHASSIS_RQ_3508_ID = 0x204,
} can_msg_id_e;

// 定义3508电机在数组里的编号
#define CHASSIS_L_Q_Motor_3508_ID 0
#define CHASSIS_L_H_Motor_3508_ID 1
#define CHASSIS_R_H_Motor_3508_ID 2
#define CHASSIS_R_Q_Motor_3508_ID 3

// 定义DM6220电机在数组里的编号
#define CHASSIS_L_Q_DM6220_ID 0
#define CHASSIS_L_H_DM6220_ID 1
#define CHASSIS_R_H_DM6220_ID 2
#define CHASSIS_R_Q_DM6220_ID 3

// DM6220机械零位（位置范围 ±12.5，需要实际测量后填写）
#define L_Q_DM6220_Middle_Pos 0.0f
#define L_H_DM6220_Middle_Pos 0.0f
#define R_H_DM6220_Middle_Pos 0.0f
#define R_Q_DM6220_Middle_Pos 0.0f

// 底盘信息   待测
#define CHASSIS_OFFSET_X 0.18f
#define CHASSIS_OFFSET_Y 0.2f
#define CHASSIS_OFFSET_L 0.269072f

#define PI               3.1415f
#define WHEEL_RADIUS     0.065f
#define WHEEL_PERIMETER  0.408f

#define pitch            0
#define yaw              1

/***********************************************CHASSIS 3508 MOTOR************************************************/
// 速度环
#define Speed_3508_KP                             0.00f
#define Speed_3508_KI                             0.00f
#define Speed_3508_KD                             0.00f

#define Speed_3508_Deadband                       0.0f
#define Speed_3508_KI_Effective_Err               0.0f
#define Speed_3508_KD_Effective_Max_Delta_Measure 0.0f
#define Speed_3508_Iout_Max                       0.00f
#define Speed_3508_PID_Out_Max                    0.00f

/*******************************************************************************************************************/

/************************************************CHASSIS DM6220 MOTOR*************************************************/
// 角度（位置）环
#define Angle_DM6220_KP                             0.00f
#define Angle_DM6220_KI                             0.00f
#define Angle_DM6220_KD                             0.00f

#define Angle_DM6220_Deadband                       0.0f
#define Angle_DM6220_KI_Effective_Err               0.0f
#define Angle_DM6220_KD_Effective_Max_Delta_Measure 0.0f

#define Angle_DM6220_Iout_Max                       0.00f
#define Angle_DM6220_PID_Out_Max                    0.00f

// 速度环
#define Speed_DM6220_KP                             0.00f
#define Speed_DM6220_KI                             0.00f
#define Speed_DM6220_KD                             0.00f

#define Speed_DM6220_Deadband                       0.0f
#define Speed_DM6220_KI_Effective_Err               0.0f
#define Speed_DM6220_KD_Effective_Max_Delta_Measure 0.0f

#define Speed_DM6220_Iout_Max                       0.00f
#define Speed_DM6220_PID_Out_Max                    0.00f

// DM6220控制参数（位置模式下使用）
#define DM6220_KP  0.0f // 位置环KP
#define DM6220_KD  0.0f // 位置环KD
#define DM6220_TOR 0.0f // 扭矩

/*******************************************************************************************************************/

void AGV_chassis_task(void);
void AGV_classis_Pid_data_Init(void);
void AGV_chassis_Init(void);
void AGV_chassis_Zero_Check(float Tar_Angle, float *Acl_Angle, float max);
void RemoteControl(void);
void loop_f(float *angle, float max);
void AGV_angle_calc(AGV_chassis_speed_Typedef *speed, float *out_angle);
void AGV_speed_calc(AGV_chassis_speed_Typedef *speed, int16_t *out_speed);
void chassis_control(void);
int8_t Find_min_Angle(float *tar_angle, float act_angle);
void DM6220_Init(void);
void AGV_Reverse_Check(float *tar_pos, float *cur_pos);

#endif /* __AGV_CHASSIS_TASK_H__ */
