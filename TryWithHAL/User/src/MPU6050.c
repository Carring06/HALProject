#include "stm32f1xx_hal.h"
#include "MPU6050_Reg.h"
#include "i2c.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

/**
 * @brief  MPU6050写寄存器
 * @param  RegAddress: 寄存器地址
 * @param  Data: 要写入的数据
 * @retval 无
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    // HAL库硬件I2C写操作：自动处理START/STOP时序，无需手动等待事件
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

/**
 * @brief  MPU6050读寄存器
 * @param  RegAddress: 寄存器地址
 * @retval 读取到的寄存器数据
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    // HAL库硬件I2C读操作：自动处理重复START/STOP时序
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
    return Data;
}

/**
 * @brief  MPU6050初始化
 * @note   底层I2C硬件初始化已由CubeMX自动生成，此处仅配置MPU6050寄存器
 * @retval 无
 */
void MPU6050_Init(void)
{
    /* MPU6050寄存器初始化（与原逻辑完全一致） */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1：取消休眠，选择X轴陀螺仪为时钟源
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2：所有轴不休眠
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器：配置采样率
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器：配置DLPF滤波
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置：选择量程±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置：选择量程±16g
}

/**
 * @brief  MPU6050获取ID
 * @retval MPU6050的WHO_AM_I寄存器值（固定为0x68）
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief  MPU6050获取传感器数据
 * @param  AccX/AccY/AccZ: 加速度计X/Y/Z轴数据指针
 * @param  GyroX/GyroY/GyroZ: 陀螺仪X/Y/Z轴数据指针
 * @retval 无
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;

    // 加速度计数据读取
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;

    // 陀螺仪数据读取
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}
