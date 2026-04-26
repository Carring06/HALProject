/**
 ******************************************************************************
 * @file    Remote_Control.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   遥控器数据处理函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 待测试
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "New_Remote_Control.h"
#include "stm32h7xx.h"
#include "math.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
Remote_Info_Typedef remote_ctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};


__attribute__((section (".AXI_SRAM"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Static Fun -------------------------------------------------------------- */
// static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/* Functions --------------------------------------------------------------- */
/**
 * @brief  SBUS数据解析至遥控器
 * @param  sbus_buf: SBUS数据缓冲区指针
 * @param  remote_ctrl: 遥控器信息结构体指针
 * @return 无
 * @note   无
 */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

		if((sbus_buf[0] == 0x0F) && (sbus_buf[24] == 0x00))
		{
    remote_ctrl->rc.ch[0] = (((uint16_t)sbus_buf[1])|((uint16_t)sbus_buf[2] << 8)) & 0x07FF;                        
    remote_ctrl->rc.ch[1] = (((uint16_t)sbus_buf[2] >> 3)|((uint16_t)sbus_buf[3] << 5)) & 0x07FF;                     
    remote_ctrl->rc.ch[2] = (((uint16_t)sbus_buf[3] >> 6)|((uint16_t)sbus_buf[4] << 2)|(uint16_t)sbus_buf[5] << 10) & 0x07FF;                         
    remote_ctrl->rc.ch[3] = (((uint16_t)sbus_buf[5] >> 1)|((uint16_t)sbus_buf[6] << 7)) & 0x07FF;  
    remote_ctrl->rc.ch[4] = (((uint16_t)sbus_buf[6] >> 4)|((uint16_t)sbus_buf[7] << 4)) & 0x07FF;                         
		remote_ctrl->rc.ch[5] = (((uint16_t)sbus_buf[7] >> 7)|((uint16_t)sbus_buf[8] << 1)|(uint16_t)sbus_buf[9] << 9) & 0x07FF;

		remote_ctrl->rc.s[0] = (((uint16_t)sbus_buf[9] >> 2)|((uint16_t)sbus_buf[10] << 6)) & 0x07FF;
	  remote_ctrl->rc.s[1] = (((uint16_t)sbus_buf[10] >> 5)|((uint16_t)sbus_buf[11] << 3)) & 0x07FF;
		remote_ctrl->rc.s[2] = (((uint16_t)sbus_buf[12] >> 5)|((uint16_t)sbus_buf[13] << 8)) & 0x07FF;
		remote_ctrl->rc.s[3] = (((uint16_t)sbus_buf[13] >> 3)|((uint16_t)sbus_buf[14] << 5)) & 0x07FF;
	            

    remote_ctrl->rc.ch[0] -= RC_CH1_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH2_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH3_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH4_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH5_VALUE_OFFSET;
    remote_ctrl->rc.ch[5] -= RC_CH6_VALUE_OFFSET;
			
			
		
		remote_ctrl->online_cnt = 0xFAU;
		remote_ctrl->rc_lost = false;
		}
}

/**
 * @brief  遥控器离线检测
 * @param  remote_ctrl: 遥控器信息结构体指针
 * @return 无
 * @note   无
 */
void Remote_Offline_Detect( Remote_Info_Typedef  *remote_ctrl)
{
    if(remote_ctrl->online_cnt <= 0x32U)
    {
        memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));
        remote_ctrl->rc_lost = true;
    }
    else if(remote_ctrl->online_cnt > 0)
    {
        remote_ctrl->online_cnt--;
    }
}

/**
 * @brief  遥控器活动检测
 * @param  remote_ctrl: 遥控器信息结构体指针
 * @return 无
 * @note   无
 */
void Remote_Active_Detect( Remote_Info_Typedef  *remote_ctrl)
{
    for(uint8_t i = 0; i < 5; i++) 
    {
        if(abs(remote_ctrl->rc.ch[i]) >= RC_CH_DEADZONE) {
            remote_ctrl->rc_active[i] = true;
            remote_ctrl->Last_Remote_Active_Time[i] = HAL_GetTick();
        }
        else remote_ctrl->rc_active[i] = false;
    }
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
