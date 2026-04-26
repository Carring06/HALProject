/**
 ******************************************************************************
 * @file    DJI_Motor.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   DJI电机驱动函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "DJI_Motor.h"
#include "fdcan.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "string.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
extern FDCAN_RxHeaderTypeDef RxHeader1;
extern uint8_t g_Can1RxData[64];

extern FDCAN_RxHeaderTypeDef RxHeader2;
extern uint8_t g_Can2RxData[64];

extern FDCAN_RxHeaderTypeDef RxHeader3;
extern uint8_t g_Can3RxData[64];

extern DJI_Motor_Info_Typedef chassis_motor_info[8];
extern DJI_Motor_Info_Typedef gimbal_motor_info[2];

/* Static Fun -------------------------------------------------------------- */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Torque_Ratio,uint16_t MAXEncoder);

/* Functions --------------------------------------------------------------- */
/**
 * @brief  大疆电机控制
 * @param  DJI_Motor: DJI电机信息结构体指针
 * @param  hcan: CAN句柄
 * @param  delay_time: 延时时间
 * @return 无
 * @note   无
 */
void DJI_Motor_ctrl(DJI_Motor_Info_Typedef *DJI_Motor,hcan_t* hcan, uint16_t delay_time)
{
	uint8_t data[8];

	data[0] = (uint8_t)((DJI_Motor[0].Data.SET_Current >> 8) & 0xFF);
	data[1] = (uint8_t)(DJI_Motor[0].Data.SET_Current & 0xFF);
	data[2] = (uint8_t)((DJI_Motor[1].Data.SET_Current >> 8) & 0xFF);
	data[3] = (uint8_t)(DJI_Motor[1].Data.SET_Current & 0xFF);
	data[4] = (uint8_t)((DJI_Motor[2].Data.SET_Current >> 8) & 0xFF);
	data[5] = (uint8_t)(DJI_Motor[2].Data.SET_Current & 0xFF);
	data[6] = (uint8_t)((DJI_Motor[3].Data.SET_Current >> 8) & 0xFF);
	data[7] = (uint8_t)(DJI_Motor[3].Data.SET_Current & 0xFF);

	canx_send_data(hcan, DJI_Motor[0].ID_Set.TxIdentifier, data, 8);
	osDelay(delay_time);
}

/**
 * @brief  大疆电机控制
 * @param  DJI_Motor: DJI电机信息结构体指针
 * @param  hcan: CAN句柄
 * @param  delay_time: 延时时间
 * @return 无
 * @note   和上面的基本一样，只是方便用于6020电机
 */
void DJI_Motor_ctrl_6020(DJI_Motor_Info_Typedef *DJI_Motor,hcan_t* hcan, uint16_t delay_time)
{
	uint8_t data[8];

	data[0] = (uint8_t)((DJI_Motor[4].Data.SET_Current >> 8) & 0xFF);
	data[1] = (uint8_t)(DJI_Motor[4].Data.SET_Current & 0xFF);
	data[2] = (uint8_t)((DJI_Motor[5].Data.SET_Current >> 8) & 0xFF);
	data[3] = (uint8_t)(DJI_Motor[5].Data.SET_Current & 0xFF);
	data[4] = (uint8_t)((DJI_Motor[6].Data.SET_Current >> 8) & 0xFF);
	data[5] = (uint8_t)(DJI_Motor[6].Data.SET_Current & 0xFF);
	data[6] = (uint8_t)((DJI_Motor[7].Data.SET_Current >> 8) & 0xFF);
	data[7] = (uint8_t)(DJI_Motor[7].Data.SET_Current & 0xFF);

	canx_send_data(hcan, DJI_Motor[4].ID_Set.TxIdentifier, data, 8);
	osDelay(delay_time);
}

/**
 * @brief  更新大疆电机信息
 * @param  DJI_Motor: DJI电机信息结构体指针
 * @param  rx_data: 接收数据指针
 * @param  data_len: 数据长度
 * @return 无
 * @note   无
 */
void DJI_Motor_Info_Update(DJI_Motor_Info_Typedef *DJI_Motor,uint8_t *rx_data,uint32_t data_len)
{
	/* 检查标识符 */
	if(data_len!=FDCAN_DLC_BYTES_8) return;
	
	/* 转换通用电机数据 */
	DJI_Motor->Data.Temperature = rx_data[6];
	DJI_Motor->Data.Encoder  = ((int16_t)rx_data[0] << 8 | (int16_t)rx_data[1]);
	DJI_Motor->Data.Velocity = ((int16_t)rx_data[2] << 8 | (int16_t)rx_data[3]);
	DJI_Motor->Data.Current  = ((int16_t)rx_data[4] << 8 | (int16_t)rx_data[5]);

	/* 将编码器值转换为角度 */
	switch(DJI_Motor->Motor_Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,DJI_6020_Ratio,8192); 	//6020减速比为1:1，输出轴为位置
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,DJI_3508_Ratio,8192);
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,DJI_2006_Ratio,8192);
		break;
		default:break;
	}
}

/**
  * @brief  循环约束
  * @param  Input: 输入值
  * @param  Min_Value: 最小值
  * @param  Max_Value: 最大值
  * @return 约束后的值
  * @note   无
  */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
  if (Max_Value < Min_Value)
  {
    return Input;
  }
  
  float len = Max_Value - Min_Value;    

  if (Input > Max_Value)
  {
      do{
          Input -= len;
      }while (Input > Max_Value);
  }
  else if (Input < Min_Value)
  {
      do{
          Input += len;
      }while (Input < Min_Value);
  }
  return Input;
}

/* Private functions ------------------------------------------------------- */
/**
 * @brief  编码器值转累计角度
 * @param  Data: DJI电机数据结构体指针
 * @param  Torque_Ratio: 减速比
 * @param  MAXEncoder: 编码器最大值
 * @return 累计角度(°) 
 * @note   无
 */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
  float res1 = 0,res2 =0;
  
  if(Data == NULL) return 0;
  
  /* 判断电机是否初始化 */
  if(Data->Initlized != true)
  {
    /* 更新上一次的编码器值 */
    Data->Last_Encoder = Data->Encoder;

    /* 重置角度 */
    Data->Angle = 0;

    /* 设置初始化标志 */
    Data->Initlized = true;
  }
  
  /* 获取可能的最小编码器误差 */
  if(Data->Encoder < Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
  }
  else if(Data->Encoder > Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
  }
  res2 = Data->Encoder - Data->Last_Encoder;
  
  /* 更新上一次的编码器值 */
  Data->Last_Encoder = Data->Encoder;
  
  /* 将编码器数据转换为总角度 */
	if(fabsf(res1) > fabsf(res2))
	{
		Data->Angle += (float)res2/(MAXEncoder*Torque_Ratio)*360.f;
	}
	else
	{
		Data->Angle += (float)res1/(MAXEncoder*Torque_Ratio)*360.f;
	}
  
  return Data->Angle;
}

/* Interrupt functions ----------------------------------------------------- */
/**
 * @brief  FDCAN RxFifo0回调函数
 * @param  hfdcan: FDCAN句柄
 * @param  RxFifo0ITs: RxFifo0中断标志
 * @return 无
 * @note   在FDCAN1和FDCAN3接收到新消息时调用
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
		/* 从RX_FIFO0检索Rx消息 */
		memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//清空接收缓冲区	
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			
		switch(RxHeader1.Identifier)
		{			
      //can1接收6020电机数据
      case Chassis_6020_Motor1_RxID://0x205 数组4
      case Chassis_6020_Motor2_RxID:
      case Chassis_6020_Motor3_RxID:
      case Chassis_6020_Motor4_RxID:
      case 0x201://云台pitch轴电机
      case 0x202://云台yaw轴电机
      {
        static uint8_t i = 0;
        // 根据ID更新对应电机数据
        i = RxHeader1.Identifier - Chassis_3508_Motor1_RxID;
        //更新电机数据
        if(i > 3)
        {
          DJI_Motor_Info_Update(&chassis_motor_info[i],g_Can1RxData,RxHeader1.DataLength);//4-7
        }else if(i < 2)
        {
          DJI_Motor_Info_Update(&gimbal_motor_info[i],g_Can1RxData,RxHeader1.DataLength);//0-1
        }
        break;
      }

			// case 3 :DM_Motor_Info_Update(&chassis_move.joint_motor[2], g_Can2RxData,RxHeader2.DataLength);break;
			// case 4 :DM_Motor_Info_Update(&chassis_move.joint_motor[3], g_Can2RxData,RxHeader2.DataLength);break;	  
			default: break;
		}			
	}
	else if(hfdcan->Instance == FDCAN3)
	{
		/* 从RX_FIFO0检索Rx消息 */
		memset(g_Can3RxData, 0, sizeof(g_Can3RxData));	// 清空接收缓冲区	
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
			
		switch(RxHeader3.Identifier)
		{

			default: break;
		}
	}
  }
}

/**
 * @brief  FDCAN RxFifo1回调函数
 * @param  hfdcan: FDCAN句柄
 * @param  RxFifo1ITs: RxFifo1中断标志
 * @return 无
 * @note   在FDCAN2接收到新消息时调用
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN2)
    {
		/* 从RX_FIFO1检索Rx消息 */
		memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
		switch(RxHeader2.Identifier)
		{
      			//can3接收3508电机数据
      case Chassis_3508_Motor1_RxID://0x201 数组0
      case Chassis_3508_Motor2_RxID:
      case Chassis_3508_Motor3_RxID:
      case Chassis_3508_Motor4_RxID:
      {
        static uint8_t j = 0;
        // 根据ID更新对应电机数据
        j = RxHeader2.Identifier - Chassis_3508_Motor1_RxID;
        //更新电机数据
        if(j < 4)
        {
         DJI_Motor_Info_Update(&chassis_motor_info[j],g_Can2RxData,RxHeader2.DataLength);
        }
        break;
      }

			// case 3 :DM_Motor_Info_Update(&chassis_move.joint_motor[2], g_Can2RxData,RxHeader2.DataLength);break;
			// case 4 :DM_Motor_Info_Update(&chassis_move.joint_motor[3], g_Can2RxData,RxHeader2.DataLength);break;	         	
			default: break;
		}	
    }
  }
}

/* ------------------------------------------------------------------------- */
