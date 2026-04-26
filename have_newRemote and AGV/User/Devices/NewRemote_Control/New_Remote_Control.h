/**
 ******************************************************************************
 * @file    Remote_Control.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   遥控器数据处理函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 待测试
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef NEW_REMOTE_CONTROL_H
#define NEW_REMOTE_CONTROL_H

/* Includes ----------------------------------------------------------------- */
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

/* Defines ------------------------------------------------------------------ */
#define SBUS_RX_BUF_NUM		25u			/* SBUS接收数据长度 */

#define RC_CH1_VALUE_OFFSET	1042U		/* 遥控器通道数据偏移 */
#define RC_CH2_VALUE_OFFSET	1024U
#define RC_CH3_VALUE_OFFSET	1024U
#define RC_CH4_VALUE_OFFSET	1028U
#define RC_CH5_VALUE_OFFSET	1024U
#define RC_CH6_VALUE_OFFSET	1024U

/* 遥控器拨杆状态 */
/*SWC拨杆共三个状态*/
#define RC_SWC_UP 						((uint16_t) 7)   					/* 遥控器开关(上) */
#define RC_SWC_MID 						((uint16_t) 1024)					/* 遥控器开关(中) */
#define RC_SWC_DOWN 					((uint16_t) 1792)					/* 遥控器开关(下) */

/*SWA,SWB,SWD拨杆均为两个状态*/
#define RC_SW_UP 							((uint16_t) 240)  
#define RC_SW_DOWN 						((uint16_t) 1087)  

#define RC_CH_DEADZONE		5U			/* 遥控器通道死区 */
/* 遥控器控制灵敏度 */
#define RC_CONTROL_SENSITIVITY	0.7f		
/* 遥控器拨杆 */

#define RC_R_Horizontal  			 	(remote_ctrl.rc.ch[0])
#define RC_R_Vertical    			 	(remote_ctrl.rc.ch[1])
#define RC_L_Vertical			 	 		(remote_ctrl.rc.ch[2])
#define RC_L_Horizontal   			(remote_ctrl.rc.ch[3])
#define Auxiliary_Channel1		 	(remote_ctrl.rc.ch[4])
#define Auxiliary_Channel2		 	(remote_ctrl.rc.ch[5])
#define RC_SWA									(remote_ctrl.rc.s[0])
#define RC_SWB									(remote_ctrl.rc.s[1])
#define RC_SWC									(remote_ctrl.rc.s[2])
#define RC_SWD									(remote_ctrl.rc.s[3])




/**
 * @brief 遥控器信息结构体
 */
typedef  struct
{
	struct
	{
		int16_t ch[6];
		uint16_t s[4];
		
	} rc;	


	bool rc_lost;			/* 丢失标志 */
	bool rc_active[5];		/* 活动标志 */
	uint8_t online_cnt;		/* 在线计数 */
	uint32_t Last_Remote_Active_Time[5];
} Remote_Info_Typedef;

/* Externs ------------------------------------------------------------------ */
extern Remote_Info_Typedef remote_ctrl;
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Functions ---------------------------------------------------------------- */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
void Remote_Active_Detect( Remote_Info_Typedef  *remote_ctrl);
void Remote_Offline_Detect( Remote_Info_Typedef  *remote_ctrl);

/* -------------------------------------------------------------------------- */
#endif /* REMOTE_CONTROL_H */
