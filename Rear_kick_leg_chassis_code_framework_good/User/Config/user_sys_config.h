#ifndef __USER_SYS_CONFIG_H
#define __USER_SYS_CONFIG_H

#include "cmsis_os.h"
#include "user_sys_config.h"
#include <stm32h7xx.h>


/* 系统DEBUG标志位 */
#define DEBUG						0
/* 系统延时启动时长(ms) */
#define SYS_DELAY_START_TIME		5000

/* Rad 转 Ang*/
#define RAD_TO_ANG					(180.f / PI)
/* Ang 转 Rad */
#define ANG_TO_RAD					(PI / 180.f)
/* 底盘速度限制(m/s) */
#define Chassis_Spd_Limit  			3.3f	

#define Chassis_Wz_Limit			4.3f

#define Chassis_angle_limit			0.01f
/* 左右关节电机限角 */
#define Chassis_Leg_Angle_min		(-0.7)
#define Chassis_Leg_Angle_max		(0.7)

/* 达妙电机零点保存 */
#define Save_Motor_Zero_flag		0

#endif
