#include "MyCAN.h"
#include "can.h"

void MyCAN_Init(void)
{
    CAN_FilterInit();
    HAL_CAN_Start(&hcan);
}

void CAN_FilterInit(void)
{
    CAN_FilterTypeDef canFilter;
    canFilter.FilterActivation = ENABLE;
    canFilter.FilterBank = 0;
    canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK)
    {
        Error_Handler();
    }
}

void MyCAN_SendData(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef canTxHeader;
    uint32_t txMailbox; // 存储发送邮箱号

    canTxHeader.StdId = id;                   // 标准CAN ID
    canTxHeader.ExtId = 0;                    // 扩展ID置0（使用标准帧）
    canTxHeader.RTR = CAN_RTR_DATA;           // 数据帧（非远程帧）
    canTxHeader.IDE = CAN_ID_STD;             // 标准帧格式
    canTxHeader.DLC = len;                    // 数据长度（1~8字节）
    canTxHeader.TransmitGlobalTime = DISABLE; // 不记录发送时间
	
   if (HAL_CAN_AddTxMessage(&hcan, &canTxHeader, data, &txMailbox) != HAL_OK)
   {
       return; 
   }

//    uint32_t timeout = HAL_GetTick() + 1000; // 1s超时
//    while (HAL_CAN_IsTxMessagePending(&hcan, txMailbox) == SET)
//    {

//        if (HAL_GetTick() > timeout)
//        { 
//             break;
//        }    
//    }
}

//直接调用HAL_CAN_GetRxMessage函数即可，不需要再次封装
// void MyCAN_RecvData(uint32_t id, uint8_t *data, uint8_t len)
// {
//     CAN_RxHeaderTypeDef canRxHeader;
//     uint8_t rxData[8];
//     canRxHeader.StdId = id;                   // 标准CAN ID
//     canRxHeader.ExtId = 0;                    // 扩展ID置0（使用标准帧）
//     canRxHeader.RTR = CAN_RTR_DATA;           // 数据帧（非远程帧）
//     canRxHeader.IDE = CAN_ID_STD;             // 标准帧格式
//     canRxHeader.DLC = len;                    // 数据长度（1~8字节）

//     if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &canRxHeader, rxData) != HAL_OK)
//     {
//         return; 
//     }
// }
