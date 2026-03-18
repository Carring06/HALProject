/* Includes ------------------------------------------------------------------------------------------- */

#include "DJI_Motor.h"
#include "fdcan.h"
#include "stdbool.h"
#include "cmsis_os.h"

/* Global variable ------------------------------------------------------------------------------------- */

// 底盘驱动轮电机
DJI_Motor_Info_Typedef DJI_Chassis_Wheel_Motor[4] = {

    [0] = {	
        .Motor_Type = DJI_M3508,
		.ID_Set = {
			.TxIdentifier = Chassis_3508_MotorA_TxID,
			.RxIdentifier = Chassis_3508_MotorR_RxID,
		}
    },
    [1] = {	
        .Motor_Type = DJI_M3508,
		.ID_Set = {
			.TxIdentifier = Chassis_3508_MotorA_TxID,
			.RxIdentifier = Chassis_3508_MotorL_RxID,
		}
    },
    [2] = {0},
    [3] = {0},

};

/* Functions -------------------------------------------------------------------------------------------- */

//  编码器值转化为角度(累加最到到float最大值)
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
  float res1 = 0,res2 =0;
  
  if(Data == NULL) return 0;
  
  /* Judge the motor Initlized */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle = 0;

    /* Set the init flag */
    Data->Initlized = true;
  }
  
  /* get the possiable min Encoder err */
  if(Data->Encoder < Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
  }
  else if(Data->Encoder > Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
  }
  res2 = Data->Encoder - Data->Last_Encoder;
  
  /* update the last Encoder */
  Data->Last_Encoder = Data->Encoder;
  
  /* transforms the Encoder data to tolangle */
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

// 大疆电机控制函数
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

// 更新大疆电机信息
void DJI_Motor_Info_Update(DJI_Motor_Info_Typedef *DJI_Motor,uint8_t *rx_data,uint32_t data_len)
{
	/* check the Identifier */
	if(data_len!=FDCAN_DLC_BYTES_8) return;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.Temperature = rx_data[6];
	DJI_Motor->Data.Encoder  = ((int16_t)rx_data[0] << 8 | (int16_t)rx_data[1]);
	DJI_Motor->Data.Velocity = ((int16_t)rx_data[2] << 8 | (int16_t)rx_data[3]);
	DJI_Motor->Data.Current  = ((int16_t)rx_data[4] << 8 | (int16_t)rx_data[5]);

	/* transform the Encoder to angle */
	switch(DJI_Motor->Motor_Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,DJI_6020_Ratio,8192); 	//6020电机减速比为1:1 拥有绝对位置
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
  * @brief  浮环约束
  * @param  Input	 指定的变量
  * @param  minValue 指定变量的最小个数
  * @param  maxValue 指定变量的最大个数
  * @retval 变量
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


/* ---------------------------------------------------------------------------------------------------- */
