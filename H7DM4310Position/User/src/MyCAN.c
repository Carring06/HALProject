#include "MyCAN.h"


void MyFDCAN_Init(void)
{
    FDCAN1_FilterInit();
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}   

void FDCAN1_FilterInit(void)
{
    /*SID & FilterMask == FilterID*/
    FDCAN_FilterTypeDef FDCAN1_FilterConfig;
    FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN1_FilterConfig.FilterIndex = 0;
    FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN1_FilterConfig.FilterID1 = 0x00;
    FDCAN1_FilterConfig.FilterID2 = 0x00;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

void MyFDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = len;             //感觉这样应该可以了

    // if (len <= 8)
    //     pTxHeader.DataLength = len;
    // else if (len == 12)
    //     pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
    // else if (len == 16)
    //     pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
    // else if (len == 20)
    //     pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
    // else if (len == 24)
    //     pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
    // else if (len == 32)
    //     pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
    // else if (len == 48)
    //     pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    // else if (len == 64)
    //     pTxHeader.DataLength = FDCAN_DLC_BYTES_64;

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
    {
        return;
    }
}

//直接调用HAL_FDCAN_GetRxMessage函数即可，不需要再次封装

