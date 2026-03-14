/***************************************************************************************
  * 本程序由江协科技创建并开源共享！（HAL库自改版） 
  * 你可以随意查看、使用和修改，并应用到自己的项目之中
  * 程序版权归江协科技所有，任何个人或组织不得将其占为己有
  * 
  * 程序名称：			NRF24L01无线通信模块驱动程序
  * 程序创建时间：		2025.6.9
  * 当前程序版本：		V1.0
  * 当前版本发布时间：	2024.6.9
  * 
  * 江协科技官方网站：	jiangxiekeji.com
  * 江协科技官方淘宝店：	jiangxiekeji.taobao.com
  * 程序介绍及更新动态：	jiangxiekeji.com/tutorial/nrf24l01.html
  * 
  * 如果你发现程序中的格式或语法错误，可以通过邮件向我们反馈：feedback@jiangxiekeji.com
  * 发送邮件之前，你可以先到更新动态页面查看最新程序，如果该问题已经修复，则无需再发邮件
  ***************************************************************************************/

// 替换标准库头文件为HAL库头文件
#include "stm32f1xx_hal.h"
#include "NRF24L01_Define.h"

/*将CE、CSN、SCK、MOSI引脚初始化为推挽输出模式*/
/*将MISO引脚初始化为上拉输入模式*/

#define  GPIO_CE			GPIO_PIN_8				/*By GPIOA*/
#define  GPIO_CSN	 		GPIO_PIN_15				/*By GPIOA*/
#define  GPIO_SCK			GPIO_PIN_3				/*By GPIOB*/
#define  GPIO_MISO			GPIO_PIN_4				/*By GPIOB*/
#define  GPIO_MOSI			GPIO_PIN_5				/*By GPIOB*/




/*全局变量*********************/

/*发送部分*/
uint8_t NRF24L01_TxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};		//发送地址，长度5字节
#define NRF24L01_TX_PACKET_WIDTH		4							//发送数据包宽度，范围：1~32字节
uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];				//发送数据包

/*接收部分*/
uint8_t NRF24L01_RxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};		//接收通道0地址，长度5字节
#define NRF24L01_RX_PACKET_WIDTH		4							//接收通道0数据包宽度，范围：1~32字节
uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];				//接收数据包

/**
  * 提示：设置A和设置B进行通信
  * A发送时，A的发送地址、发送数据包宽度要与B的接收地址、接收数据包宽度对应相同
  * B发送时，B的发送地址、发送数据包宽度要与A的接收地址、接收数据包宽度对应相同
  * 通常情况下，可以把A和B的发送地址、接收地址设置成一样，A和B的发送数据包宽度、接收数据包宽度也全部设置成一样
  * 这样A和B可以使用完全一样的模块程序，操作更加方便，也不容易出错
  * 
  */

/*********************全局变量*/


/*引脚配置*********************/

/**
  * 函数名：NRF24L01写CE高低电平
  * 参数：要写入CE的电平值，范围：0/1
  * 返回值：无
  * 说明：替换为HAL库GPIO操作
  */
void NRF24L01_W_CE(uint8_t BitValue)
{
	// HAL库：BitValue=1时输出高电平，0时输出低电平
	HAL_GPIO_WritePin(GPIOA, GPIO_CE, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函数名：NRF24L01写CSN高低电平
  * 参数：要写入CSN的电平值，范围：0/1
  * 返回值：无
  * 说明：替换为HAL库GPIO操作
  */
void NRF24L01_W_CSN(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_CSN, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函数名：NRF24L01写SCK高低电平
  * 参数：要写入SCK的电平值，范围：0/1
  * 返回值：无
  * 说明：替换为HAL库GPIO操作
  */
void NRF24L01_W_SCK(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_SCK, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函数名：NRF24L01写MOSI高低电平
  * 参数：要写入MOSI的电平值，范围：0/1
  * 返回值：无
  * 说明：替换为HAL库GPIO操作
  */
void NRF24L01_W_MOSI(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_MOSI, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函数名：NRF24L01读MISO高低电平
  * 参数：无
  * 返回值：读取到的MISO的电平值，范围：0/1
  * 说明：替换为HAL库GPIO操作
  */
uint8_t NRF24L01_R_MISO(void)
{
	// HAL库：读取引脚电平，高电平返回1，低电平返回0
	return HAL_GPIO_ReadPin(GPIOA, GPIO_MISO) == GPIO_PIN_SET ? 1 : 0;
}

/**
  * 函数名：NRF24L01引脚初始化
  * 参数：无
  * 返回值：无
  * 说明：CubeMX已完成GPIO时钟使能和引脚初始化，此处仅设置默认电平
  */
void NRF24L01_GPIO_Init(void)
{
	// 【CubeMX已自动生成以下代码，无需手动编写】
	// 1. GPIOA时钟自动使能

	
	// 仅保留默认电平设置，与原逻辑一致
	NRF24L01_W_CE(0);		//CE默认为0，关闭收发模式
	NRF24L01_W_CSN(1);		//CSN默认为1，不选中从机
	NRF24L01_W_SCK(0);		//SCK默认为0，对应SPI模式0
	NRF24L01_W_MOSI(0);		//MOSI默认电平任意，0和1均可
}

/*********************引脚配置*/


/*通信协议*********************/

/**
  * 函数名：SPI交换一个字节
  * 参数：Byte 要发送的一个字节数据，范围：0x00~0xFF
  * 返回值：接收到的一个字节数据，范围：0x00~0xFF
  * 说明：完全保留原逻辑，未做任何修改
  */
uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte)
{
	uint8_t i;
	
	/*此处使用SPI模式0进行通信*/
	/*循环8次，主机依次发出和收进数据的每一位*/
	for (i = 0; i < 8; i ++)
	{
		/*SPI高位先行，因此发出高位到MOSI引脚*/
		if (Byte & 0x80)			//判断Byte的最高位
		{
			NRF24L01_W_MOSI(1);		//如果是1，则向MOSI输出1
		}
		else
		{
			NRF24L01_W_MOSI(0);		//如果是0，则向MOSI输出0
		}
		Byte <<= 1;					//Byte左移一位，最低位空出来用于接收数据位
		
		/*产生SCK上升沿*/
		NRF24L01_W_SCK(1);
		
		/*从MISO引脚读入数据，存到Byte的最低位*/
		if (NRF24L01_R_MISO())		//读取MISO引脚
		{
			Byte |= 0x01;			//如果是1，则给Byte最低位置1
		}							//如果是0，则不做任何操作，因为左移后低位默认是0
		
		/*产生SCK下降沿*/
		NRF24L01_W_SCK(0);
	}
	
	/*返回Byte数据，此时的Byte为SPI交换接收到的一个字节数据*/
	return Byte;
}

/*********************通信协议*/


/*指令实现*********************/

/**
  * 函数名：NRF24L01读取寄存器（一个字节）
  * 参数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 返回值：指定寄存器的数据，范围：0x00~0xFF
  * 说明：完全保留原逻辑
  */
uint8_t NRF24L01_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	
	/*发送读寄存器指令后，开始交换接收，得到指定地址的数据*/
	Data = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
	
	/*返回读到的一个字节数据*/
	return Data;
}

/**
  * 函数名：NRF24L01读取寄存器（多个字节）
  * 参数：RegAddress 指定寄存器的地址，范围：0x00~0x1F
  * 参数：DataArray 读取到的数据数组，输出参数
  * 参数：Count 指定读取的数量，范围：1~5
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	
	/*发送读寄存器指令后，开始交换接收，循环接收多次，得到指定地址下的多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将接收到的数据写入到输出参数DataArray中*/
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函数名：NRF24L01写入寄存器（一个字节）
  * 参数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 参数：Data 要写入的一个字节数据，范围：0x00~0xFF
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	
	/*发送写寄存器指令后，开始交换发送，在指定地址下写入数据*/
	NRF24L01_SPI_SwapByte(Data);
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函数名：NRF24L01写入寄存器（多个字节）
  * 参数：RegAddress 指定寄存器的地址，范围：0x00~0x1F
  * 参数：DataArray 要写入的数据数组，输入参数
  * 参数：Count 指定写入的数量，范围：1~5
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	
	/*发送写寄存器指令后，开始交换发送，循环发送多次，在指定地址下写入多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将输入参数DataArray的数据写入到指定地址中*/
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函数名：NRF24L01读取Rx有效载荷
  * 参数：DataArray 读取到的数据数组，输出参数
  * 参数：Count 指定读取的数量，范围：1~32
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_ReadRxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读取Rx有效载荷*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_RX_PAYLOAD);
	
	/*发送读取Rx有效载荷指令后，开始交换接收，循环接收多次，得到多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将读取的数据写入到输出参数DataArray中*/
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函数名：NRF24L01写入Tx有效载荷
  * 参数：DataArray 要写入的数据数组，输入参数
  * 参数：Count 指定写入的数量，范围：1~5
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_WriteTxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写入Tx有效载荷*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_TX_PAYLOAD);
	
	/*发送写入Tx有效载荷指令后，开始交换发送，循环发送多次，写入多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将输入参数DataArray的数据写入到Tx有效载荷中*/
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函数名：NRF24L01清空Tx FIFO的所有数据
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_FlushTx(void)
{
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，清空Tx FIFO*/
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_TX);
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函数名：NRF24L01清空Rx FIFO的所有数据
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_FlushRx(void)
{
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，清空Rx FIFO*/
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_RX);
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函数名：NRF24L01读取状态寄存器
  * 参数：无
  * 返回值：状态寄存器的值，范围：0x00~0xFF
  * 说明：完全保留原逻辑
  */
uint8_t NRF24L01_ReadStatus(void)
{
	uint8_t Status;
	
	/*CSN拉低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，空指令*/
	/*第一个字节发送任意指令，都可以交换得到状态寄存器的值*/
	Status = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	
	/*CSN拉高，通信结束*/
	NRF24L01_W_CSN(1);
	
	/*返回状态寄存器的值*/
	return Status;
}

/*********************指令实现*/


/*功能函数*********************/

/**
  * 函数名：NRF24L01进入掉电模式（CE = 0，PWR_UP = 0）
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_PowerDown(void)
{
	uint8_t Config;
	
	/*CE拉低，关闭收发模式*/
	NRF24L01_W_CE(0);
	
	/*读改写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config &= ~0x02;								//配置寄存器位1（PWR_UP）置0
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
}

/**
  * 函数名：NRF24L01进入待机模式1（CE = 0，PWR_UP = 1）
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_StandbyI(void)
{
	uint8_t Config;
	
	/*CE拉低，关闭收发模式*/
	NRF24L01_W_CE(0);
	
	/*读改写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x02;									//配置寄存器位1（PWR_UP）置1
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
}

/**
  * 函数名：NRF24L01进入接收模式（CE = 1，PWR_UP = 1，PRIM_RX = 1）
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_Rx(void)
{
	uint8_t Config;
	
	/*CE拉低，关闭收发模式*/
	NRF24L01_W_CE(0);
	
	/*读改写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x03;									//配置寄存器位1（PWR_UP）和位0（PRIM_RX）都置1
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
	
	/*CE置1，进入收发模式，因为PRIM_RX为1，所以进入接收模式*/
	NRF24L01_W_CE(1);
}

/**
  * 函数名：NRF24L01进入发送模式（CE = 1，PWR_UP = 1，PRIM_RX = 0）
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_Tx(void)
{
	uint8_t Config;
	
	/*CE拉低，关闭收发模式*/
	NRF24L01_W_CE(0);
	
	/*读改写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x02;									//配置寄存器位1（PWR_UP）置1
	Config &= ~0x01;								//配置寄存器位0（PRIM_RX）置0
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
	
	/*CE置1，进入收发模式，因为PRIM_RX为0，所以进入发送模式*/
	NRF24L01_W_CE(1);
}

/**
  * 函数名：NRF24L01初始化
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑，仅底层GPIO初始化已由CubeMX完成
  */
void NRF24L01_Init(void)
{
	/*调用底层的端口初始化（仅设置默认电平，CubeMX已完成硬件配置）*/
	NRF24L01_GPIO_Init();
	
	/*初始化配置一系列寄存器，寄存器值的含义需参考手册中的寄存器描述*/
	/*以下配置通信双方必须保持一致，否则无法进行通信*/
	NRF24L01_WriteReg(NRF24L01_CONFIG, 0x08);		//配置寄存器，不屏蔽中断，使能CRC，CRC为1字节，PWR_UP = 0，PRIM_RX = 0
	NRF24L01_WriteReg(NRF24L01_EN_AA, 0x3F);		//使能自动应答，开启接收通道0~通道5的自动应答
	NRF24L01_WriteReg(NRF24L01_EN_RXADDR, 0x01);	//使能接收通道，只开启接收通道0
	NRF24L01_WriteReg(NRF24L01_SETUP_AW, 0x03);		//设置地址宽度，地址宽度为5字节
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0x03);	//设置自动重传，间隔250us，重传3次
	NRF24L01_WriteReg(NRF24L01_RF_CH, 0x02);		//射频通道，频率为(2400 + 2)MHz = 2.402GHz
	NRF24L01_WriteReg(NRF24L01_RF_SETUP, 0x0E);		//射频设置，通信速率为2Mbps，发射功率为0dBm
	
	/*接收通道0的数据包宽度，设置为宏定义NRF24L01_RX_PACKET_WIDTH指定的值*/
	NRF24L01_WriteReg(NRF24L01_RX_PW_P0, NRF24L01_RX_PACKET_WIDTH);
	
	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*清空Rx FIFO的所有数据*/
	NRF24L01_FlushRx();
	
	/*给状态寄存器的位4（MAX_RT）、位5（TX_DS）和位6（RX_DR）写1，清除中断位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
	
	/*初始化配置完成，模块默认进入接收模式*/
	NRF24L01_Rx();
}

/**
  * 函数名：NRF24L01发送数据包
  * 参数：无
  * 返回值：发送标志位，方便用户了解发送状态
  * 			1：发送成功，无错误
  * 			2：达到了最大重发次数仍未收到应答，可能是收发双方配置不一致、接收方不存在、接收FIFO已满或者多个发送数据包冲突
  * 			3：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			4：发送超时，可能是模块未初始化、断路、短路或者引脚配置不正确
  * 说明：完全保留原逻辑
  */
uint8_t NRF24L01_Send(void)
{
	uint8_t Status;
	uint8_t SendFlag;
	uint32_t Timeout;
	
	/*发送地址，设置为全局数组NRF24L01_TxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_TxAddress, 5);
	
	/*接收通道0地址，此处必须也设置为发送地址，用于接收应答*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_TxAddress, 5);
	
	/*写入发送有效载荷，写入全局数组NRF24L01_TxPacket指定的数据，数据宽度为NRF24L01_TX_PACKET_WIDTH*/
	NRF24L01_WriteTxPayload(NRF24L01_TxPacket, NRF24L01_TX_PACKET_WIDTH);
	
	/*发送的地址和有效载荷写入完成，进入发送模式，开始发送数据*/
	NRF24L01_Tx();
	
	/*指定超时时间，即循环读取状态寄存器的次数，具体值可以实测确定*/
	Timeout = 10000;
	
	/*循环读取状态寄存器*/
	while (1)
	{
		/*读取状态寄存器，保存到Status变量*/
		Status = NRF24L01_ReadStatus();
		
		/*超时计数*/
		Timeout --;
		if (Timeout == 0)			//如果计数减到0
		{
			SendFlag = 4;			//发送超时，设标志位为4
			NRF24L01_Init();		//发送出错，重新初始化一次模块，这样有助于模块从错误中恢复正常
			break;					//跳出循环
		}
		
		/*根据状态寄存器的值，判断发送状态*/
		if ((Status & 0x30) == 0x30)		//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
		{
			SendFlag = 3;			//状态寄存器的值不合法，设中断位为3
			NRF24L01_Init();		//发送出错，重新初始化一次模块，这样有助于模块从错误中恢复正常
			break;					//跳出循环
		}
		else if ((Status & 0x10) == 0x10)	//状态寄存器位4（MAX_RT）为1
		{
			SendFlag = 2;			//达到了最大重发次数仍未收到应答，设标志位为2
			NRF24L01_Init();		//发送出错，重新初始化一次模块，这样有助于模块从错误中恢复正常
			break;					//跳出循环
		}
		else if ((Status & 0x20) == 0x20)	//状态寄存器位5（TX_DS）为1
		{
			SendFlag = 1;			//发送成功，无错误，设标志位为1
			break;					//跳出循环
		}
	}
	
	/*给状态寄存器的位4（MAX_RT）和位5（TX_DS）写1，清除中断位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x30);
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*发送完成后，恢复接收通道0原来的地址*/
	/*如果发送地址和接收通道0地址设置相同，则可不执行这一步*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
	
	/*发送完成，模块恢复为接收模式*/
	NRF24L01_Rx();
		
	/*返回发送标志位*/
	return SendFlag;
}

/**
  * 函数名：NRF24L01接收数据包
  * 参数：无
  * 返回值：接收标志位，方便用户了解接收状态
  * 			0：未接收到数据包
  * 			1：成功接收到一个数据包
  * 			2：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			3：模块仍处于掉电模式，可能是模块未初始化、电源断电过、断路、短路或者引脚配置不正确
  * 说明：完全保留原逻辑
  */
uint8_t NRF24L01_Receive(void)
{
	uint8_t Status, Config;
	uint8_t ReceiveFlag;
	
	/*读取状态寄存器，保存到Status变量*/
	Status = NRF24L01_ReadStatus();
	
	/*读取配置寄存器，保存到Config变量*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	
	/*根据配置寄存器和状态寄存器的值，判断接收状态*/
	if ((Config & 0x02) == 0x00)		//配置寄存器位1（PWR_UP）为0
	{
		ReceiveFlag = 3;				//设备仍处于掉电模式，设标志位为3
		NRF24L01_Init();				//接收出错，重新初始化一次模块，这样有助于模块从错误中恢复正常
	}
	else if ((Status & 0x30) == 0x30)	//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
	{
		ReceiveFlag = 2;				//状态寄存器的值不合法，设中断位为2
		NRF24L01_Init();				//接收出错，重新初始化一次模块，这样有助于模块从错误中恢复正常
	}
	else if ((Status & 0x40) == 0x40)	//状态寄存器位6（RX_DR）为1
	{
		ReceiveFlag = 1;				//接收到数据，设标志位为1
		
		/*读取接收有效载荷，存储在全局数组NRF24L01_RxPacket中，数据宽度为NRF24L01_RX_PACKET_WIDTH*/
		NRF24L01_ReadRxPayload(NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);
		
		/*给状态寄存器的位6（RX_DR）写1，清除中断位*/
		NRF24L01_WriteReg(NRF24L01_STATUS, 0x40);

		/*清空Rx FIFO的所有数据*/
		NRF24L01_FlushRx();
	}
	else
	{
		ReceiveFlag = 0;				//未接收到数据，设中断位为0
	}
	
	/*返回接收标志位*/
	return ReceiveFlag;
}

/**
  * 函数名：NRF24L01更新接收地址
  * 参数：无
  * 返回值：无
  * 说明：完全保留原逻辑
  */
void NRF24L01_UpdateRxAddress(void)
{
	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
}

/*********************功能函数*/


/*****************江协科技|版权所有***************/
/*****************jiangxiekeji.com*****************/

