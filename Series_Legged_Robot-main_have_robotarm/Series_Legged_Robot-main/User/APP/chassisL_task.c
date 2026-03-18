/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can2总线上
	*						 从底盘上往下看，左上角的DM4310发送id为8、接收id为4，
	*						 左下角的DM4310发送id为6、接收id为3，
	*						 左边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisL_task.h"
#include "fdcan.h"
#include "VMC_calc.h"
#include "Remote_Control.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "VOFA.h"
#include "user_lib.h"

vmc_leg_t left;

float LQR_K_L[12]={  
  -3.3259  , -0.2272,   -1.2959,   -1.2122 ,   3.3133,    0.3249,
    2.5219 ,   0.2142 ,   1.8118 ,   1.6073 ,   8.2857 ,   0.4180
};

extern float Poly_Coefficient[12][4];
extern chassis_t chassis_move;

extern float spin_angle_end; 			// 最终旋转角度
extern float spin_Leg;					// 旋转长度目标

uint8_t Spin_legL_finish_flag = 0;		//左腿旋转完成标志位
extern uint8_t Spin_legR_finish_flag;	//右腿旋转完成标志位
float jump_time2;
uint32_t CHASSL_TIME=1;	

Ramp_Typedef Ramp_L;

Joint_Motor_t joint1;
Joint_Motor_t joint2;
Joint_Motor_t joint3;
Joint_Motor_t joint4;
Joint_Motor_t joint5;

float joint_pos[5] = {};
float joint_ves[5] = {};
void ChassisL_task(void)
{
	joint_motor_init(&joint1,1,POS_MODE);//发送id为6
	joint_motor_init(&joint2,2,POS_MODE);//发送id为6
	joint_motor_init(&joint3,3,POS_MODE);//发送id为6
	joint_motor_init(&joint3,4,POS_MODE);//发送id为6
	joint_motor_init(&joint5,5,POS_MODE);//发送id为6
	joint_pos[0] = 0.0f;
	joint_pos[1] = 1.8f;
	joint_pos[2] = -1.8f;
	joint_pos[3] = 0.0f;
	joint_pos[4] = 0.0f;
	enable_motor_mode(&hfdcan2,4,POS_MODE);
		  osDelay(1);
		enable_motor_mode(&hfdcan2,4,POS_MODE);
		  osDelay(1);
		enable_motor_mode(&hfdcan2,4,POS_MODE);
	
	while(1)
	{	
			if(remote_ctrl.rc.s[1] == 3)
				{
					enable_motor_mode(&hfdcan2,0x01,POS_MODE);
					osDelay(1);
					enable_motor_mode(&hfdcan2,0x02,POS_MODE);
					osDelay(1);
					enable_motor_mode(&hfdcan2,0x03,POS_MODE);
					osDelay(1);
					enable_motor_mode(&hfdcan2,0x04,POS_MODE);
					osDelay(1);
					enable_motor_mode(&hfdcan2,0x05,POS_MODE);
					osDelay(1);
				}
		if(remote_ctrl.rc.s[1] == 1)
			{
		joint_pos[0]	=joint_pos[0] + ((float)remote_ctrl.rc.ch[4]/660)*(-0.008f);
		VAL_LIMIT(joint_pos[0],0,2.8f);
				
		joint_pos[1]	=joint_pos[1] + ((float)remote_ctrl.rc.ch[3]/660)*(-0.008f);
		VAL_LIMIT(joint_pos[1],1.8f,6.9f);
				
		joint_pos[2]	=joint_pos[2] + ((float)remote_ctrl.rc.ch[1]/660)*(-0.008f);
		VAL_LIMIT(joint_pos[2],-4.5f,-1.8f);
				
		joint_pos[3]	=joint_pos[3] + ((float)remote_ctrl.rc.ch[2]/660)*(-0.008f);
		VAL_LIMIT(joint_pos[3],-1.57f,1.57f);
				
		joint_pos[4] 	=joint_pos[4] + ((float)remote_ctrl.rc.ch[0]/660)*(-0.008f);
		
		joint_ves[0] = 3.0f;
		joint_ves[1] = 5.0f;
		joint_ves[2] = 5.0f;
		joint_ves[3] = 3.0f;
		joint_ves[4] = 3.0f;
		pos_speed_ctrl(&hfdcan2,0x01,joint_pos[0],joint_ves[0]);
				  osDelay(1);
		pos_speed_ctrl(&hfdcan2,0x02,joint_pos[1],joint_ves[1]);
				  osDelay(1);
		pos_speed_ctrl(&hfdcan2,0x03,joint_pos[2],joint_ves[2]);
				  osDelay(1);
		pos_speed_ctrl(&hfdcan2,0x04,joint_pos[3],joint_ves[3]);
				  osDelay(1);
			//	pos_speed_ctrl(&hfdcan2,0x04,1,2);
		}
	}
}















