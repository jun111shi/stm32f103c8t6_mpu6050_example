#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

// 读取MPU6050寄存器
uint8_t MPU6050_ReadReg(I2C_HandleTypeDef *hi2c,uint8_t reg);
// 写入MPU6050寄存器
void MPU6050_WriteReg(I2C_HandleTypeDef *hi2c,uint8_t reg, uint8_t value);
// 初始化MPU6050
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
// 读取加速度数据
void MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);
// 读取角速度数据
void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
// 读取温度数据
int16_t MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c);
// 转换温度数据
float MPU6050_ConvertTemp(int16_t raw_temp);
// 计算角度
void CalculateAngles(int16_t accel_x, int16_t accel_y, int16_t accel_z, float *roll, float *pitch);
//void MPU6050_Init(I2C_HandleTypeDef *hi2c);
//void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, double *roll, double *pitch);

#endif /* MPU6050_H */
