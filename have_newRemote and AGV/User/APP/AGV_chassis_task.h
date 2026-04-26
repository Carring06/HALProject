#ifndef __AGV_CHASSIS_TASK_H__
#define __AGV_CHASSIS_TASK_H__   

#include "main.h"

//底盘速度结构体
typedef struct
{
    float vx; // 底盘速度x轴 m/s 
    float vy; // 底盘速度y轴 m/s
    float vw; // 底盘角速度 rad/s
} AGV_chassis_speed_Typedef;

//底盘速度结构体
typedef struct
{
    float vyaw; // 云台yaw rad/s 
    float vpitch; // 云台pitch rad/s 
} AGV_gimbal_ctrl_Typedef;

//系统运行状态
typedef enum
{
    Initing=0,
    Running=1,	
}SystemValue;

//底盘电机ID枚举
typedef enum
{
    CAN_CHASSIS_3508_ID = 0x200,
    CAN_CHASSIS_LQ_3508_ID = 0x201,
    CAN_CHASSIS_LH_3508_ID = 0x202,
    CAN_CHASSIS_RH_3508_ID = 0x203,
    CAN_CHASSIS_RQ_3508_ID = 0x204,

	CAN_CHASSIS_LQ_6020_ID = 0x205, 
    CAN_CHASSIS_LH_6020_ID = 0x206,
    CAN_CHASSIS_RH_6020_ID = 0x207,
    CAN_CHASSIS_RQ_6020_ID = 0x208,
    CAN_CHASSIS_6020_ID = 0x1FF
} can_msg_id_e;

//定义电机在数组里的编号
#define CHASSIS_L_Q_Motor_3508_ID 0
#define CHASSIS_L_H_Motor_3508_ID 1
#define CHASSIS_R_H_Motor_3508_ID 2
#define CHASSIS_R_Q_Motor_3508_ID 3
#define CHASSIS_L_Q_Motor_6020_ID 4
#define CHASSIS_L_H_Motor_6020_ID 5
#define CHASSIS_R_H_Motor_6020_ID 6
#define CHASSIS_R_Q_Motor_6020_ID 7

//机械零位一定要给个0~4096中间的,记得
#define L_Q_6020_Middle_ECD 7509.0f   
#define L_H_6020_Middle_ECD 6144.0f
#define R_H_6020_Middle_ECD 4779.0f
#define R_Q_6020_Middle_ECD 2048.0f

//底盘信息
#define CHASSIS_OFFSET_X  0.18f
#define CHASSIS_OFFSET_Y  0.2f
#define CHASSIS_OFFSET_L  0.269072f 

#define PI  3.1415f 
#define WHEEL_RADIUS  0.065f   //0.269
#define WHEEL_PERIMETER   0.408f//周长

#define pitch 0
#define yaw 1

/***********************************************CHASSIS 3508 MOTOR************************************************/
//角度（位置）
#define Angle_KP 					            	2.0f		
#define Angle_KI 					            	0.0002f			 
#define Angle_KD 					            	0.0021f      		 

#define Angle_KI_Effective_Max_Err                  20.0f
#define Angle_KD_Effective_Max_Err                  300.0        
#define Angle_Target								0   //期望角度(位置)   
//#define Angle_Err_Max								0	
#define Angle_Err_Min						    	5.0f//死区带宽
#define Angle_Iout_Max								50	//积分限幅    
#define Angle_PID_Out_Max					        1200.0f //输出限幅
		
//速度		
#define Speed_PID_Mode      			            0	               
#define Speed_3508_KP 							    13.0f	      
#define Speed_3508_KI 							    0.03f      
#define Speed_3508_KD 								.0f

//当速度误差大于下面的数时，关闭Ki项作用或者Kd项作用,一般是用于区分角度环和速度环,速度环完全可以不用这两条公式
#define Speed_3508_Deadband                         0.0f
#define Speed_3508_KI_Effective_Err                 0.0f    //0.0f表示关闭该有效积分功能,表示无论误差多少都积分
#define Speed_3508_KD_Effective_Max_Delta_Measure   0.0f    //表示关闭
  
#define Speed_3508_Iout_Max							1500.0f //积分限幅      
#define Speed_3508_PID_Out_Max						16000.f	 //输出限幅
						

/*******************************************************************************************************************/

/************************************************CHASSIS 6020 MOTOR*************************************************/

//角度（位置）
#define Angle_6020_KP 		                    	1.0f				
#define Angle_6020_KI 	                    		0.0f				 
#define Angle_6020_KD 		                    	0.0f	   		 

#define Angle_6020_Deadband							20.0f	//死区带宽	//误差在1度以内
#define Angle_6020_KI_Effective_Err    	    		1000.0f //50.0f
#define Angle_6020_KD_Effective_Max_Delta_Measure   50.0f			        

#define Angle_6020_Iout_Max							2.0f	//积分限幅    
#define Angle_6020_PID_Out_Max					    250.f//25000.0f//给这么大相当于不限幅//输出限幅
		
//速度			               
#define Speed_6020_KP                           	80.0f		      
#define Speed_6020_KI                           	0.1f        
#define Speed_6020_KD 	                            0.0f

#define Speed_6020_Deadband			            	0.f	 
#define Speed_6020_KI_Effective_Err                 5000.f
#define Speed_6020_KD_Effective_Max_Delta_Measure   0.0f
			
#define Speed_6020_Iout_Max							8000.0f //积分限幅      
#define Speed_6020_PID_Out_Max						25000.f	//输出限幅

/*******************************************************************************************************************/

void AGV_chassis_task(void);

void AGV_classis_Pid_data_Init(void);
void AGV_chassis_Init(void);
void AGV_chassis_Zero_Check(int16_t Tar_Angle,int16_t *Acl_Angle,int16_t max);
void RemoteControl(void);
void loop_f(float *angle,float max);
void AGV_speed_calc(AGV_chassis_speed_Typedef *speed,int16_t *out_speed);
void chassis_control(void);
int8_t Find_min_Angle(float *tar_angle,float act_angle);
void AGV_Gimbal_Init(void);
void gimbal_control(void);


#endif /* __AGV_CHASSIS_TASK_H__ */
