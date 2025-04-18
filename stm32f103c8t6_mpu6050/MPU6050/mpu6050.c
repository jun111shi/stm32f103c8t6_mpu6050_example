#include "mpu6050.h"
#include <math.h>

//#define WHO_AM_I_REG 0x75
//#define PWR_MGMT_1_REG 0x6B
//#define SMPLRT_DIV_REG 0x19
//#define ACCEL_CONFIG_REG 0x1C
//#define GYRO_CONFIG_REG 0x1B
//#define ACCEL_XOUT_H_REG 0x3B

#define RAD_TO_DEG 57.295779513082320876798154814105

// MPU6050 I2C地址
#define MPU6050_ADDR 0xD0
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43


// 读取MPU6050寄存器
uint8_t MPU6050_ReadReg(I2C_HandleTypeDef *hi2c,uint8_t reg) 
{
    uint8_t value;
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}

// 写入MPU6050寄存器
void MPU6050_WriteReg(I2C_HandleTypeDef *hi2c,uint8_t reg, uint8_t value) 
{
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}


// 初始化MPU6050
void MPU6050_Init(I2C_HandleTypeDef *hi2c) 
{
    MPU6050_WriteReg(hi2c,MPU6050_PWR_MGMT_1, 0x00); // 退出睡眠模式
}


// 读取加速度数据
void MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) 
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);
    *accel_x = (data[0] << 8) | data[1];
    *accel_y = (data[2] << 8) | data[3];
    *accel_z = (data[4] << 8) | data[5];
}


// 读取角速度数据
void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) 
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);
    *gyro_x = (data[0] << 8) | data[1];
    *gyro_y = (data[2] << 8) | data[3];
    *gyro_z = (data[4] << 8) | data[5];
}

// 读取温度数据
int16_t MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c) 
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY) != HAL_OK) 
		{
        return 0; // 返回一个错误值
    }
    int16_t temp = (data[0] << 8) | data[1];
    return temp;
}
// 转换温度数据
float MPU6050_ConvertTemp(int16_t raw_temp)
{
    return (raw_temp / 340.0f) + 36.53f;
}

// 计算角度
void CalculateAngles(int16_t accel_x, int16_t accel_y, int16_t accel_z, float *roll, float *pitch)
{
    float ax = accel_x / 16384.0f;
    float ay = accel_y / 16384.0f;
    float az = accel_z / 16384.0f;

    *roll = atan2f(ay, az) * RAD_TO_DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

//void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
//    uint8_t i2cBuf[2];

//    // 唤醒 MPU6050
//    i2cBuf[0] = PWR_MGMT_1_REG;
//    i2cBuf[1] = 0x00;
//    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, i2cBuf, 2, 10);

//    // 设置采样率
//    i2cBuf[0] = SMPLRT_DIV_REG;
//    i2cBuf[1] = 0x07;
//    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, i2cBuf, 2, 10);

//    // 配置加速度计和陀螺仪
//    i2cBuf[0] = ACCEL_CONFIG_REG;
//    i2cBuf[1] = 0x00;  // ±2g
//    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, i2cBuf, 2, 10);
//    i2cBuf[0] = GYRO_CONFIG_REG;
//    i2cBuf[1] = 0x00;  // ±250°/s
//    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, i2cBuf, 2, 10);
//}

//void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, double *roll, double *pitch) {
//    uint8_t i2cBuf[6];
//    int16_t ax, ay, az;

//    i2cBuf[0] = ACCEL_XOUT_H_REG;
//    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, i2cBuf, 1, 10);
//    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, i2cBuf, 6, 10);

//    ax = (int16_t)(i2cBuf[0] << 8 | i2cBuf[1]);
//    ay = (int16_t)(i2cBuf[2] << 8 | i2cBuf[3]);
//    az = (int16_t)(i2cBuf[4] << 8 | i2cBuf[5]);

//    *roll = atan2(ay, az) * RAD_TO_DEG;
//    *pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
//}
