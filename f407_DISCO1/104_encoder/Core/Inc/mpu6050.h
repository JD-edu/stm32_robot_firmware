/*
 * mpu6050.h
 *
 *  Created on: Sep 6, 2025
 *      Author: conne
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

extern I2C_HandleTypeDef hi2c1;

#define MPU6050_ADDR 0xD0 // MPU6050 I2C 8-bit address (AD0 pin connected to GND)
#define WHO_AM_I_REG 0x75  // WHO_AM_I register address
#define PWR_MGMT_1    0x6B // Power Management 1 register
#define ACCEL_CONFIG  0x1C // Accelerometer Configuration register
#define GYRO_CONFIG	  0x1B // Gyroscope Configuration register
#define SMPLRT_DIV    0x19 // Sample Rate Divider register
#define CONFIG        0x1A // Configuration register
#define ACCEL_XOUT_H  0x3B  // Accelerometer X-axis data high byte address
#define GYRO_XOUT_H   0x43 // Gyroscope X-axis data high byte address

uint8_t Read_MPU6050_ID(); // Reads MPU6050's WHO_AM_I register
void MPU6050_Write(uint8_t reg, uint8_t data); // Writes a single byte to an MPU6050 register
void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length); // Reads multiple bytes from an MPU6050 register
void MPU6050_Init(void); // Initializes the MPU6050 sensor
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az); // Reads accelerometer data
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz); // Reads gyroscope data

#endif /* INC_MPU6050_H_ */
