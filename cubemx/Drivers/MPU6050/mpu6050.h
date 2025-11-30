/*
 * mpu6050.h
 *
 *  Created on: Jul 18, 2025
 *      Author: ryans
 */

#ifndef MPU6050_MPU6050_H_
#define MPU6050_MPU6050_H_

#include "stm32f4xx_hal.h"

#define MPU6050_ADDRESS (0x68 << 1)
#define MPU6050_REG_PWR_MGMT 0x6B
#define MPU6050_REG_ACCEL_X 0x3B

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_ReadAccelGyro(I2C_HandleTypeDef *hi2c, int16_t *data);

#endif /* MPU6050_MPU6050_H_ */
