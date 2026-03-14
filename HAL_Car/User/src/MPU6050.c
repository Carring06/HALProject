#include "stm32f1xx_hal.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C设备地址

/**
 * 函数功能：MPU6050写寄存器
 * 输入参数：RegAddress 寄存器地址，范围：参考MPU6050相关寄存器定义
 * 输入参数：Data 要写入寄存器的数据，范围：0x00~0xFF
 * 返回值：无
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C开始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送设备地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答位
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答位
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答位
	MyI2C_Stop();						//I2C停止
}

/**
 * 函数功能：MPU6050读寄存器
 * 输入参数：RegAddress 寄存器地址，范围：参考MPU6050相关寄存器定义
 * 返回值：读取寄存器的数据，范围：0x00~0xFF
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();						//I2C开始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送设备地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答位
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答位
	MyI2C_Start();						//I2C重复开始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//发送设备地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答位
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，对从机非应答以终止从机的数据发送
	MyI2C_Stop();						//I2C停止
	
	return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;

	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();

	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	for (i = 0; i < Count; i++)
	{
		DataArray[i] = MyI2C_ReceiveByte();
		if (i < Count - 1)
		{
			MyI2C_SendAck(0);
		}
		else
		{
			MyI2C_SendAck(1);
		}
	}
	MyI2C_Stop();
}

/**
 * 函数功能：MPU6050初始化
 * 输入参数：无
 * 返回值：无
 */
void MPU6050_Init(void)
{
	MyI2C_Init();									//先初始化软件I2C
	
	/*MPU6050寄存器初始化，需要对MPU6050相关寄存器定义配置，此处仅配置几个关键的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消休眠模式，选择时钟源为X轴陀螺
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值，所有轴不休眠
	// MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	// MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);		// 1ms刷新一次
	MPU6050_WriteReg(MPU6050_CONFIG, 0x00);			// 去掉滤波
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺配置寄存器，选择量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度传感器配置寄存器，选择量程为±16g
}

/**
 * 函数功能：MPU6050获取ID号
 * 输入参数：无
 * 返回值：MPU6050的ID号
 */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
 * 函数功能：MPU6050获取数据
 * 输入参数：AccX AccY AccZ 加速度X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
 * 输入参数：GyroX GyroY GyroZ 陀螺X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
 * 返回值：无
 */
// void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
// 						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
// {
// 	uint8_t DataH, DataL;								//定义数据高八位和低八位的变量
	
// 	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度X轴的高八位数据
// 	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度X轴的低八位数据
// 	*AccX = (DataH << 8) | DataL;						//数据合并后，通过输出参数返回
	
// 	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度Y轴的高八位数据
// 	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度Y轴的低八位数据
// 	*AccY = (DataH << 8) | DataL;						//数据合并后，通过输出参数返回
	
// 	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度Z轴的高八位数据
// 	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度Z轴的低八位数据
// 	*AccZ = (DataH << 8) | DataL;						//数据合并后，通过输出参数返回
	
// 	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺X轴的高八位数据
// 	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺X轴的低八位数据
// 	*GyroX = (DataH << 8) | DataL;						//数据合并后，通过输出参数返回
	
// 	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺Y轴的高八位数据
// 	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺Y轴的低八位数据
// 	*GyroY = (DataH << 8) | DataL;						//数据合并后，通过输出参数返回
	
// 	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺Z轴的高八位数据
// 	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺Z轴的低八位数据
// 	*GyroZ = (DataH << 8) | DataL;						//数据合并后，通过输出参数返回
// }

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
					 int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t Data[14];

	MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);

	*AccX = (Data[0] << 8) | Data[1];
	*AccY = (Data[2] << 8) | Data[3];
	*AccZ = (Data[4] << 8) | Data[5];

	*GyroX = (Data[8] << 8) | Data[9];
	*GyroY = (Data[10] << 8) | Data[11];
	*GyroZ = (Data[12] << 8) | Data[13];
}
