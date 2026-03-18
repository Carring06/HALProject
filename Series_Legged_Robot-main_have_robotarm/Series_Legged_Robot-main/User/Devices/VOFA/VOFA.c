/* Includes ------------------------------------------------------------------------------------------- */
#include "VOFA.h"
#include "usart.h"
#include <string.h>
/* Defines -------------------------------------------------------------------------------------------- */

/* Global variable ------------------------------------------------------------------------------------- */
float vofa_data[15] = {0};
/* Static Fun ------------------------------------------------------------------------------------------- */
static void Float_to_Byte(float Fdata,  uint8_t *ArrayByte)
{
    Vofa_Typedef Vofa;                  //定义Vofa_Typedef类型的Vofa变量
    
    Vofa.fval= Fdata;               	//把需要操作的浮点数复制到共同体的Fdata变量中
    ArrayByte[0] = Vofa.uval;       	//0-7位移到数组元素0
    ArrayByte[1] = Vofa.uval >> 8;   	//8-15位移动到数组元素1
    ArrayByte[2] = Vofa.uval >> 16;  	//16-23位移动到数组元素2
    ArrayByte[3] = Vofa.uval >> 24;  	//24-31位移动到数组元素3 
    
}
/* Functions -------------------------------------------------------------------------------------------- */
void Vofa_JustFloat_send(float *vofa_data,uint8_t num)
{
	static uint8_t uval_temp[4];
	static uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
	static uint8_t send_buffer[256]; // 创建足够大的缓冲区
    
    uint16_t total_len = 0;
    for(uint8_t i = 0; i < num; i++)
    {
        Float_to_Byte(vofa_data[i], &send_buffer[total_len]);
        total_len += 4;
    }
    memcpy(&send_buffer[total_len], tail, 4);
    total_len += 4;

    HAL_UART_Transmit_DMA(&huart7, send_buffer, total_len);
}
/* ---------------------------------------------------------------------------------------------------- */