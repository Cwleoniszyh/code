#include "mpu6500.h"
#include "stm32f4xx_hal.h"
#include <math.h>



// Raw sensor data
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;

// Calibration offset values
float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;

// Attitude data
float angle_pitch = 0.0f, angle_roll = 0.0f, angle_yaw = 0.0f;
uint32_t last_time = 0;

// Write to MPU6500 register
static HAL_StatusTypeDef MPU6500_WriteReg(uint8_t reg, uint8_t data) {
  uint8_t tx_data[2] = {reg, data};
  return HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, tx_data, 2, 100);
}

// Read from MPU6500 register
static HAL_StatusTypeDef MPU6500_ReadReg(uint8_t reg, uint8_t *data, uint16_t len) {
  HAL_StatusTypeDef status;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, &reg, 1, 100);
  if (status == HAL_OK) {
    status = HAL_I2C_Master_Receive(&hi2c1, MPU6500_ADDR, data, len, 100);
  }
  return status;
}

// Initialize MPU6500
HAL_StatusTypeDef MPU6500_Init(void) {
  HAL_StatusTypeDef status;
  uint8_t who_am_i;
  
  // Wake up MPU6500 (exit sleep mode)
  status = MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x00);
  if (status != HAL_OK) return status;
  
  HAL_Delay(100);
  
  // Check device ID
  status = MPU6500_ReadReg(MPU6500_WHO_AM_I, &who_am_i, 1);
  if (status != HAL_OK || who_am_i != 0x70) return HAL_ERROR;
  
  // Configure gyroscope range to 2000DPS
  status = MPU6500_WriteReg(MPU6500_GYRO_CONFIG, GYRO_RANGE_2000DPS);
  if (status != HAL_OK) return status;
  
  // Configure accelerometer range to 8G
  status = MPU6500_WriteReg(MPU6500_ACCEL_CONFIG, ACCEL_RANGE_8G);
  if (status != HAL_OK) return status;
  
  // Set sample rate divider
  status = MPU6500_WriteReg(MPU6500_SMPLRT_DIV, 0x07);  // 1kHz sampling rate
  if (status != HAL_OK) return status;
  
  last_time = HAL_GetTick();
  return HAL_OK;
}

// Calibrate MPU6500
HAL_StatusTypeDef MPU6500_Calibrate(void) {
  uint16_t i;
  int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  
  // Read gyroscope data 1000 times and calculate average as offset compensation
  for (i = 0; i < 1000; i++) {
    uint8_t data[6];
    MPU6500_ReadReg(MPU6500_GYRO_XOUT_H, data, 6);
    
    gyro_x = (int16_t)((data[0] << 8) | data[1]);
    gyro_y = (int16_t)((data[2] << 8) | data[3]);
    gyro_z = (int16_t)((data[4] << 8) | data[5]);
    
    gyro_x_sum += gyro_x;
    gyro_y_sum += gyro_y;
    gyro_z_sum += gyro_z;
    
    HAL_Delay(1);
  }
  
  gyro_offset_x = (float)gyro_x_sum / 1000.0f;
  gyro_offset_y = (float)gyro_y_sum / 1000.0f;
  gyro_offset_z = (float)gyro_z_sum / 1000.0f;
  
  return HAL_OK;
}

// Read MPU6500 data
HAL_StatusTypeDef MPU6500_ReadData(void) {
  uint8_t data[14];
  HAL_StatusTypeDef status;
  
  // Read accelerometer and gyroscope data
  status = MPU6500_ReadReg(MPU6500_ACCEL_XOUT_H, data, 14);
  if (status != HAL_OK) return status;
  
  // Parse accelerometer data
  accel_x = (int16_t)((data[0] << 8) | data[1]);
  accel_y = (int16_t)((data[2] << 8) | data[3]);
  accel_z = (int16_t)((data[4] << 8) | data[5]);
  
  // Parse gyroscope data
  gyro_x = (int16_t)((data[8] << 8) | data[9]);
  gyro_y = (int16_t)((data[10] << 8) | data[11]);
  gyro_z = (int16_t)((data[12] << 8) | data[13]);
  
  // Calculate Euler angles
  MPU6500_GetEulerAngles(&angle_yaw, &angle_pitch, &angle_roll);
  
  return HAL_OK;
}

// Calculate Euler angles
void MPU6500_GetEulerAngles(float *yaw, float *pitch, float *roll) {
  uint32_t current_time = HAL_GetTick();
  float dt = (current_time - last_time) / 1000.0f;  // Time difference (seconds)
  last_time = current_time;
  
  // Convert to G units
  float ax = (float)accel_x / 4096.0f;  // 8G range: 4096 LSB/g
  float ay = (float)accel_y / 4096.0f;
  float az = (float)accel_z / 4096.0f;
  
  // Convert to °/s units
  float gx = ((float)gyro_x - gyro_offset_x) / 16.4f;  // 2000DPS range: 16.4 LSB/(°/s)
  float gy = ((float)gyro_y - gyro_offset_y) / 16.4f;
  float gz = ((float)gyro_z - gyro_offset_z) / 16.4f;
  
  // Calculate angles from accelerometer
  float accel_pitch = atan2(ax, az) * 180.0f / M_PI;
  float accel_roll = atan2(ay, az) * 180.0f / M_PI;
  
  // Complementary filter to fuse accelerometer and gyroscope data
  angle_pitch = 0.96f * (angle_pitch + gy * dt) + 0.04f * accel_pitch;
  angle_roll = 0.96f * (angle_roll + gx * dt) + 0.04f * accel_roll;
  
  // Integrate to calculate yaw angle
  angle_yaw += gz * dt;
  
  // Limit angle range to ±180°
  if (angle_yaw > 180.0f) angle_yaw -= 360.0f;
  if (angle_yaw < -180.0f) angle_yaw += 360.0f;
  
  // Assign output values
  *yaw = angle_yaw;
  *pitch = angle_pitch;
  *roll = angle_roll;
}