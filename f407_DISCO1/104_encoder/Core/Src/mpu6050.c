/*
 * mpu6050.c
 *
 *  Created on: Sep 6, 2025
 *      Author: conne
 */

#include "kalman.h"
#include "stdint.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "mpu6050.h"

uint8_t who_am_i = 0; // Stores the WHO_AM_I register value

uint8_t Read_MPU6050_ID() {
	int uga = 0;
	who_am_i =  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
	return uga;
}

void MPU6050_Write(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    if(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 2, 100) != HAL_OK){
    	printf("I2C error\n");
    }
}

void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length) {
    if(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100) != HAL_OK){
    	printf("I2C error\n");
    }
    if(HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, data, length, 100) != HAL_OK){
    	printf("I2C error\n");
    }
}

void MPU6050_Init(void) {
	// MPU6050 software reset
	 MPU6050_Write(PWR_MGMT_1, 0x80);
	 HAL_Delay(100);

    // 1️⃣ ?��?�� 모드 ?��?�� (PWR_MGMT_1)
    MPU6050_Write(PWR_MGMT_1, 0x00);
    HAL_Delay(100);

    // 2️⃣ �??????????��?�� 범위 ?��?�� (±2g)
    MPU6050_Write(ACCEL_CONFIG, 0x00);  // 00 = ±2g
    HAL_Delay(10);

    // 2️⃣ ?��?���???????? 범위 ?��?�� (±250°/s)
    MPU6050_Write(0x1B, 0x00);  // 00 = ±250°/s
    HAL_Delay(10);

    // 3️⃣ ?��?���????????? ?��?�� ?��?�� (SMPLRT_DIV)
    MPU6050_Write(SMPLRT_DIV, 0x07);  // 1kHz / (1 + 7) = 125Hz ?��?���?????????
    HAL_Delay(10);

    // 4️⃣ ???�� ?���????????? ?��?�� ?��?�� (CONFIG)
    MPU6050_Write(CONFIG, 0x03);  // 44Hz ?��?�� ?��?��
    HAL_Delay(10);
}

void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buffer[6];  // �??????????��?�� ?��?��?�� 6바이?�� (X, Y, Z)

    MPU6050_Read(ACCEL_XOUT_H, buffer, 6);

    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);  // X�?????????
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);  // Y�?????????
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);  // Z�?????????
}

void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[6];  // ?��?���???????? ?��?��?�� 6바이?�� (X, Y, Z)

    MPU6050_Read(GYRO_XOUT_H, buffer, 6);

    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);  // X�????????
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);  // Y�????????
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);  // Z�????????
}
