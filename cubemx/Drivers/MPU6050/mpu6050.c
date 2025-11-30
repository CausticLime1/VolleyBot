/*
 * mpu6050.c
 *
 *  Created on: Jul 18, 2025
 *      Author: ryans
 */

#include "mpu6050.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c){
	uint8_t data = 0;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT, 1, &data, 1, HAL_MAX_DELAY);
}

void MPU6050_ReadAccelGyro(I2C_HandleTypeDef *hi2c, int16_t *data){
	uint8_t rawData[14];
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDRESS, MPU6050_REG_ACCEL_X, 1, rawData, 14, HAL_MAX_DELAY);

	for (int i = 0; i < 7; i++){
		data[i] = (int16_t)(rawData[2 * i] << 8 | rawData[2 * i + 1]);
	}
}
