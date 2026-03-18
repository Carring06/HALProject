/* Define to prevent recursive inclusion --------------------------------------------------------------	*/

#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

/* Includes ------------------------------------------------------------------------------------------- */

#include "stdbool.h"
#include "stm32h723xx.h"
#include "can_bsp.h"
#include "pid.h"

/* Defines -------------------------------------------------------------------------------------------- */

/* 3508最大转速(rpm) */
#define MAX_3508_RPM 					8500
/* 底盘3508电机发送ID */
#define Chassis_3508_MotorA_TxID		0x200
/* 底盘3508电机[n]接收ID */
#define Chassis_3508_MotorR_RxID		0x201
#define Chassis_3508_MotorL_RxID		0x202
/* 3508电机减速比 */
#define DJI_3508_Ratio 					(268.0f/17.0f)
/* 2006电机减速比 */
#define DJI_2006_Ratio 					36.f
/* 6020电机减速比 */
#define DJI_6020_Ratio 					1.f

/* Enums ---------------------------------------------------------------------------------------------- */

// 电机类型枚举
typedef enum{
	
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

// 电机使能模式
typedef enum{
	
	Motor_Enable,
	Motor_Disable,
	Motor_Save_Zero_Position,
	DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/* Structs -------------------------------------------------------------------------------------------- */

// 电机ID结构体
typedef struct{
	
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */

}DJI_Motor_ID_Typedef;

// 电机接收参数结构体 
typedef struct {
	
	bool 	 Initlized;   		/*!< init flag */
	int16_t  SET_Current;   	/*!< Motor electric current */
	int16_t  Current;   		/*!< Motor electric current */
	int16_t  Velocity;    		/*!< Motor rotate velocity (RPM)*/
	int16_t  Encoder;   		/*!< Motor encoder angle */
	int16_t  Last_Encoder;   	/*!< previous Motor encoder angle */
	float    Angle;   			/*!< Motor angle in degree */
	uint8_t  Temperature;   	/*!< Motor Temperature */
}DJI_Motor_Data_Typedef;

// 电机信息结构体
typedef struct{
	
	DJI_Motor_Type_e Motor_Type;   				/*!< Type of Motor */
	DJI_Motor_ID_Typedef ID_Set;    			/*!< information for the CAN Transfer */
	DJI_Motor_Data_Typedef Data;   				/*!< information for the Motor Device */
	float wheel_T;								//轮毂电机的输出扭矩，单位为N
}DJI_Motor_Info_Typedef;

// 电机控制器信息结构体
typedef struct
{
	Pid_Set_Typedef Angle_set;
	Pid_Set_Typedef Speed_set;
	PidTypedef Angle_pid;
	PidTypedef Speed_pid;

} DJI_Motor_Ctrl_Typedef;

/* Externs ------------------------------------------------------------------*/

extern DJI_Motor_Info_Typedef DJI_Yaw_Motor,DJI_Chassis_Wheel_Motor[4];

/* Functions ------------------------------------------------------------------------------------------ */

void DJI_Motor_ctrl(DJI_Motor_Info_Typedef *DJI_Motor,hcan_t* hcan, uint16_t delay_time);
void DJI_Motor_Info_Update(DJI_Motor_Info_Typedef *DJI_Motor,uint8_t *rx_data,uint32_t data_len);

#endif //DEVICE_MOTOR_H
/* ---------------------------------------------------------------------------------------------------- */
