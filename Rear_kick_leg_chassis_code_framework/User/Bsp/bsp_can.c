/**
 ******************************************************************************
 * @file    bsp_can.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   CAN通信功能函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "bsp_can.h"
#include "fdcan.h"
#include "string.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DM_Motor.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t g_Can3RxData[64];

/* External variables -------------------------------------------------------- */
extern DJI_Motor_Info_Typedef chassis_motor_info[4];
extern DM_Motor_Info_Typedef chassis_dm_info[4];
extern DM_Motor_Ctrl_Typedef chassis_dm_ctrl[4];

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
 * @brief  FDCAN1过滤器和中断配置
 * @param  无
 * @return 无
 * @note   无
 */
void FDCAN1_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* 配置Rx过滤器 */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID; //标准ID
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0x00000000;

	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* 全局过滤器配置 */
	/* 接收到消息ID为标准ID，过滤器不匹配，则拒绝标准ID远程帧 */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* 激活RX FIFO0新消息通知中断 */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	
  	/* 启动FDCAN模块 */
  	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  	{
  	  Error_Handler();
  	}
}

/**
  * @brief  FDCAN2过滤器和中断配置（用于右侧DM6220 + 右侧3508）
  * @param  无
  * @return 无
  * @note   接收DM6220(0x12,0x14) + 3508(0x202,0x204)反馈
  */
void FDCAN2_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* 配置Rx过滤器：接收所有标准ID，在回调中筛选 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x00000000; // 接收所有标准ID
	sFilterConfig.FilterID2 = 0x00000000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}

	/* 全局过滤器配置 */
	/* 接收到消息ID为标准ID，过滤器不匹配，则拒绝标准ID远程帧 */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}

	/* 激活RX FIFO1新消息通知中断 */
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* 启动FDCAN模块 */
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
	  Error_Handler();
	}
}

/**
 * @brief  FDCAN3过滤器和中断配置
 * @param  无
 * @return 无
 * @note   无
 */
void FDCAN3_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* 配置Rx过滤器 */
	sFilterConfig.IdType =  FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 2;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0x00000000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* 全局过滤器配置 */
	/* 接收到消息ID为标准ID，过滤器不匹配，则拒绝标准ID远程帧 */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}

	/* 激活RX FIFO1新消息通知中断 */
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* 启动FDCAN模块 */
	if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  CAN发送数据
 * @param  hcan: CAN句柄
 * @param  id: 	 CAN ID
 * @param  data: 数据指针
 * @param  len:  数据长度
 * @return 0: 	 成功
 * @note   无
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;
	TxHeader.IdType =  FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	if(len<=8){
		TxHeader.DataLength = len<<16;
	}else if(len==12){
		TxHeader.DataLength = FDCAN_DLC_BYTES_12;
	}else if(len==16){
		TxHeader.DataLength = FDCAN_DLC_BYTES_16;
	}else if(len==20){
		TxHeader.DataLength = FDCAN_DLC_BYTES_20;
	}else if(len==24){
		TxHeader.DataLength = FDCAN_DLC_BYTES_24;	
	}else if(len==48){
		TxHeader.DataLength = FDCAN_DLC_BYTES_48;
	}else if(len==64){
		TxHeader.DataLength = FDCAN_DLC_BYTES_64;
	}
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 比特率切换关闭，仅用于经典CAN
	TxHeader.FDFormat =  FDCAN_CLASSIC_CAN; // 经典CAN
	TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0; // 消息标记

	 HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
	 return 0;
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */
/**
  * @brief  FDCAN RxFifo0回调函数（覆盖弱实现）
  * @param  hfdcan: FDCAN句柄
  * @param  RxFifo0ITs: RxFifo0中断标志
  * @note   FDCAN1用于DM6220(0x12索引1,0x13索引2) + 3508(0x202索引1,0x203索引2)
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    // 处理FDCAN1：DM6220(索引1,2) + 3508(索引1,2)
    if(hfdcan->Instance == FDCAN1)
    {
      memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);

      // DM6220转向电机反馈处理（RxID: 0x12索引1, 0x13索引2）
      switch(RxHeader1.Identifier)
      {
        case 0x12: // 索引1：DM6220 ID 0x02
        {
          DM_Motor_Info_Update(&chassis_dm_info[1], g_Can1RxData, RxHeader1.DataLength);
          break;
        }
        case 0x13: // 索引2：DM6220 ID 0x03
        {
          DM_Motor_Info_Update(&chassis_dm_info[2], g_Can1RxData, RxHeader1.DataLength);
          break;
        }
        // 3508轮毂电机反馈处理（RxID: 0x202索引1, 0x203索引2）
        case 0x202: // 索引1
        {
          DJI_Motor_Info_Update(&chassis_motor_info[1], g_Can1RxData, RxHeader1.DataLength);
          break;
        }
        case 0x203: // 索引2
        {
          DJI_Motor_Info_Update(&chassis_motor_info[2], g_Can1RxData, RxHeader1.DataLength);
          break;
        }
        // 机械臂DM电机反馈处理（如有需要可在此添加）
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
  * @note   在FDCAN2接收到新消息时调用（DM6220 0x11,0x14 + 3508 0x201,0x204）
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

		/* 处理DM6220转向电机反馈（RxID: 0x11索引0, 0x14索引3） */
		switch(RxHeader2.Identifier)
		{
			case 0x11: // 索引0：左前DM6220
			{
				DM_Motor_Info_Update(&chassis_dm_info[0], g_Can2RxData, RxHeader2.DataLength);
				break;
			}
			case 0x14: // 索引3：右前DM6220
			{
				DM_Motor_Info_Update(&chassis_dm_info[3], g_Can2RxData, RxHeader2.DataLength);
				break;
			}
			/* 处理3508轮毂电机反馈（RxID: 0x201索引0, 0x204索引3） */
			case 0x201:
			{
				DJI_Motor_Info_Update(&chassis_motor_info[0], g_Can2RxData, RxHeader2.DataLength);
				break;
			}
			case 0x204:
			{
				DJI_Motor_Info_Update(&chassis_motor_info[3], g_Can2RxData, RxHeader2.DataLength);
				break;
			}
			default: break;
		}
    }
  }
}

/* ------------------------------------------------------------------------- */
