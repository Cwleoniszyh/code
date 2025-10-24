#ifndef __MPU6500_H
#define __MPU6500_H

#include "stm32f4xx_hal.h"
#include <math.h>


extern I2C_HandleTypeDef hi2c1;  // 声明为外部变量
#define M_PI 3.14159265358979323846


// MPU6500寄存器定义
#define MPU6500_ADDR         0xD0  // I2C地址
#define MPU6500_WHO_AM_I     0x75
#define MPU6500_PWR_MGMT_1   0x6B
#define MPU6500_SMPLRT_DIV   0x19
#define MPU6500_CONFIG       0x1A
#define MPU6500_GYRO_CONFIG  0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_XOUT_H 0x3B
#define MPU6500_GYRO_XOUT_H  0x43

// 陀螺仪量程
typedef enum {
  GYRO_RANGE_250DPS  = 0x00,
  GYRO_RANGE_500DPS  = 0x08,
  GYRO_RANGE_1000DPS = 0x10,
  GYRO_RANGE_2000DPS = 0x18
} GyroRange;

// 加速度计量程
typedef enum {
  ACCEL_RANGE_2G  = 0x00,
  ACCEL_RANGE_4G  = 0x08,
  ACCEL_RANGE_8G  = 0x10,
  ACCEL_RANGE_16G = 0x18
} AccelRange;

// 函数声明
HAL_StatusTypeDef MPU6500_Init(void);
HAL_StatusTypeDef MPU6500_Calibrate(void);
HAL_StatusTypeDef MPU6500_ReadData(void);
void MPU6500_GetEulerAngles(float *yaw, float *pitch, float *roll);

#endif /* __MPU6500_H */
#ifndef __MPU6500_H
#define __MPU6500_H

#include "stm32f4xx_hal.h"
#include <math.h>


extern I2C_HandleTypeDef hi2c1;  // Declare as external variable
#define M_PI 3.14159265358979323846


// MPU6500 register definitions
#define MPU6500_ADDR         0xD0  // I2C address
#define MPU6500_WHO_AM_I     0x75
#define MPU6500_PWR_MGMT_1   0x6B
#define MPU6500_SMPLRT_DIV   0x19
#define MPU6500_CONFIG       0x1A
#define MPU6500_GYRO_CONFIG  0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_XOUT_H 0x3B
#define MPU6500_GYRO_XOUT_H  0x43

// Gyroscope range
typedef enum {
  GYRO_RANGE_250DPS  = 0x00,
  GYRO_RANGE_500DPS  = 0x08,
  GYRO_RANGE_1000DPS = 0x10,
  GYRO_RANGE_2000DPS = 0x18
} GyroRange;

// Accelerometer range
typedef enum {
  ACCEL_RANGE_2G  = 0x00,
  ACCEL_RANGE_4G  = 0x08,
  ACCEL_RANGE_8G  = 0x10,
  ACCEL_RANGE_16G = 0x18
} AccelRange;

// Function declarations
HAL_StatusTypeDef MPU6500_Init(void);
HAL_StatusTypeDef MPU6500_Calibrate(void);
HAL_StatusTypeDef MPU6500_ReadData(void);
void MPU6500_GetEulerAngles(float *yaw, float *pitch, float *roll);

#endif /* __MPU6500_H */