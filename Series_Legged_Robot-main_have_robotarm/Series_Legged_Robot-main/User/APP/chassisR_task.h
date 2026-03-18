#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "DJI_Motor.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"

#define SAVE_ZERO	0


#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)


#define ROLL_PID_KP 0.1f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT  0.2f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 15.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 1.5f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.5f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.3f
#define TURN_PID_MAX_OUT  1.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f

typedef struct
{
  Joint_Motor_t joint_motor[4];
  //DJI_Motor_Info_Typedef wheel_motor[2];
	DJI_Motor_Info_Typedef wheel_motor[2];
	
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
	float target_v;
	
	float turn_set;//期望yaw轴弧度
	float roll_set;	//期望roll轴弧度
	float roll_x;
	float phi_set;
	float theta_set;
	
	float leg_set_l;//期望腿长，单位是m
	float leg_set_r;
	float leg_set;
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
		
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
		
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志

	uint8_t jump_flag;//跳跃标志
	uint8_t jump_flag1;//右腿跳跃标志
	uint8_t jump_flag2;//左腿跳跃标志
	
	uint8_t jump_f;//左腿跳跃标志
		
	uint8_t prejump_flag;//预跳跃标志
	
	uint8_t recover_flag;//一种情况下的倒地自起标志
	
} chassis_t;

extern PidTypedef Roll_Pid;//

extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypedef *legr);
extern void ChassisR_task(void);
extern void Pensation_init(PidTypedef *roll,PidTypedef *Tp,PidTypedef *turn);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypedef *leg);

#endif




