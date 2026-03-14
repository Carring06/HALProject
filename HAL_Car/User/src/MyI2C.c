#include "Header.h"

/**
 * 函数功能：I2C总线SCL引脚电平配置
 * 输入参数：BitValue  需要写入SCL的电平，范围0~1
 * 返回值：无
 * 注意事项：当BitValue为0时，将SCL置为低电平，当BitValue为1时，将SCL置为高电平
 */
void MyI2C_W_SCL(uint8_t BitValue)
{
	// HAL库：根据BitValue配置SCL引脚电平
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
	// Delay_us(10); // 对应原代码的10us微秒延时
}

/**
 * 函数功能：I2C总线SDA引脚电平配置
 * 输入参数：BitValue  需要写入SDA的电平，范围0~1
 * 返回值：无
 * 注意事项：当BitValue为0时，将SDA置为低电平，当BitValue为1时，将SDA置为高电平
 */
void MyI2C_W_SDA(uint8_t BitValue)
{
	// HAL库：根据BitValue配置SDA引脚电平
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
	// Delay_us(10); // 对应原代码的10us微秒延时
}

/**
 * 函数功能：I2C读取SDA引脚电平
 * 输入参数：无
 * 返回值：当前SDA的电平，范围0~1，0为低电平，1为高电平
 */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	// HAL库：读取SDA引脚电平
	BitValue = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET ? 1 : 0;
	// Delay_us(10);	// 对应原代码的10us微秒延时
	return BitValue;	// 返回DA电平
}

/**
 * 函数功能：I2C初始化
 * 输入参数：无
 * 返回值：无
 * 注意事项：GPIO初始化由CubeMX自动生成，此处仅配置默认电平
 */
void MyI2C_Init(void)
{
	// 【CubeMX自动生成代码，无需手动编写】
	// 1. GPIOB时钟使能
	// 2. PB10/PB11配置为开漏输出、上拉、50MHz速度
	
	// 仅保留电平配置，与原代码一致
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	// SCL默认高电平
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);	// SDA默认高电平
}

/*软件模拟*/

/**
 * 函数功能：I2C开始
 * 输入参数：无
 * 返回值：无
 */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);							// 释放DA，确保SDA为高电平
	MyI2C_W_SCL(1);							// 释放CL，确保SCL为高电平
	MyI2C_W_SDA(0);							// 在SCL高电平期间，拉低DA，产生起始信号
	MyI2C_W_SCL(0);							// 起始后SCL也拉低，为后续总线操作，也为后续字节传输的准备
}

/**
 * 函数功能：I2C停止
 * 输入参数：无
 * 返回值：无
 */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);							// 拉低DA，确保SDA为低电平
	MyI2C_W_SCL(1);							// 释放CL，使SCL回到高电平
	MyI2C_W_SDA(1);							// 在SCL高电平期间，释放DA，产生停止信号
}

/**
 * 函数功能：I2C发送一个字节
 * 输入参数：Byte  要发送的这一个字节数据，范围：0x00~0xFF
 * 返回值：无
 */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)				// 循环8次，逐位发送数据的每一位
	{
		// 两个!可以将数据转换为布尔逻辑的表达，确保值统一转为1，即：!!(0) = 0，!!(其他) = 1
		MyI2C_W_SDA(!!(Byte & (0x80 >> i)));// 使用位运算取出Byte的指定一位数据并写入到SDA线
		MyI2C_W_SCL(1);						// 释放CL，从机在SCL高电平期间读取SDA
		MyI2C_W_SCL(0);						// 拉低CL，从机开始发送下一位数据
	}
}

/**
 * 函数功能：I2C接收一个字节
 * 输入参数：无
 * 返回值：接收到的这一个字节数据，范围：0x00~0xFF
 */
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;					// 定义接收的数据，并初始化为0x00，此处必须初始化为0x00，否则会出错
	MyI2C_W_SDA(1);							// 接收前，主机必须释放DA，以便从机的数据发送
	for (i = 0; i < 8; i ++)				// 循环8次，逐位接收从机的每一位
	{
		MyI2C_W_SCL(1);						// 释放CL，主机在SCL高电平期间读取SDA
		if (MyI2C_R_SDA()){Byte |= (0x80 >> i);}	// 读取SDA数据，并存储到Byte变量
												// 当SDA为1时，置位指定位为1，当SDA为0时，不处理指定位为默认的初始值0
		MyI2C_W_SCL(0);						// 拉低CL，从机在SCL低电平期间准备SDA
	}
	return Byte;							// 返回接收到的这一个字节数据
}

/**
 * 函数功能：I2C发送应答位
 * 输入参数：AckBit  要发送的应答位，范围：0~1，0表示应答，1表示非应答
 * 返回值：无
 */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);					// 主机将应答位数据放到SDA线上
	MyI2C_W_SCL(1);							// 释放CL，从机在SCL高电平期间，读取应答位
	MyI2C_W_SCL(0);							// 拉低CL，开始下一个时序周期
}

/**
 * 函数功能：I2C接收应答位
 * 输入参数：无
 * 返回值：接收到的应答位，范围：0~1，0表示应答，1表示非应答
*/
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;							// 定义应答位变量
	MyI2C_W_SDA(1);							// 接收前，主机必须释放DA，以便从机的数据发送
	MyI2C_W_SCL(1);							// 释放CL，主机在SCL高电平期间读取SDA
	AckBit = MyI2C_R_SDA();					// 将应答位存储到变量中
	MyI2C_W_SCL(0);							// 拉低CL，开始下一个时序周期
	return AckBit;							// 返回定义的应答位变量
}
