가속도 센서 x, y, z, 자이로 센서 x, y, 를 바이트 어레이로 시리얼 통해 pc로 보내는 코드 보여주세요.

***실제로는 내가 수정을 좀 했다*** 
```
void Send_IMU_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {

    uint8_t tx_buffer[12];

    // 16비트 정수를 바이트 배열로 변환 (Little Endian)
    tx_buffer[0] = *ax & 0xFF;
    tx_buffer[1] = (*ax >> 8) & 0xFF;
    tx_buffer[2] = *ay & 0xFF;
    tx_buffer[3] = (*ay >> 8) & 0xFF;
    tx_buffer[4] = *az & 0xFF;
    tx_buffer[5] = (*az >> 8) & 0xFF;
    tx_buffer[6] = *gx & 0xFF;
    tx_buffer[7] = (*gx >> 8) & 0xFF;
    tx_buffer[8] = *gy & 0xFF;
    tx_buffer[9] = (*gy >> 8) & 0xFF;
    tx_buffer[10] = *gz & 0xFF;
    tx_buffer[11] = (*gz >> 8) & 0xFF;

    // UART 전송
    HAL_UART_Transmit(&huart2, tx_buffer, 12, 100);
}
```

```
 while (1)
  {
	if(f_imu_send == 1){
		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
		//HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
		MPU6050_ReadAccel(&ax, &ay, &az);
		MPU6050_ReadGyro(&gx, &gy, &gz);
		//printf("Accel X: %d Y: %d Z: %d Gyro: X: %d, Y: %d, Z: %d \r\n", ax, ay, az, gx, gy, gz);
		Send_IMU_Data(&ax, &ay, &az, &gx, &gy, &gz);
		f_imu_send = 0;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

```
import serial
import struct

# 시리얼 포트 설정 (Windows: "COMx", Linux: "/dev/ttyUSBx")
ser = serial.Serial(port="COM62", baudrate=9600, timeout=1)

while True:
    data = ser.read(12)  # 10바이트 수신 (16비트 데이터 5개)

    if len(data) == 12:
        # Little Endian 형식으로 변환
        ax, ay, az, gx, gy, gz = struct.unpack('<hhhhhh', data)
        print(f"Accel X: {ax}, Y: {ay}, Z: {az}, Gyro X: {gx}, Y: {gy}  z: {gz}")
```

***참고만 할 것 ... 쳇GPT의 답변
아래는 STM32 코드 (C)와 Python 코드로 가속도 및 자이로 센서 데이터를 바이트 배열로 변환하여 UART를 통해 PC로 전송하는 예제입니다.
---

✅ 1. STM32 코드 (C, HAL Library 사용)


📌 가속도 X, Y, Z 및 자이로 X, Y 데이터를 바이트 배열로 변환 후 송신

```
#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

#define MPU6050_ADDR  (0x68 << 1)
#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43

void MPU6050_ReadAccelGyro(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy) {
    uint8_t buffer[10];

    // 가속도 (X, Y, Z) 및 자이로 (X, Y) 데이터 읽기
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 10, 100);

    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);  // Accel X
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);  // Accel Y
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);  // Accel Z
    *gx = (int16_t)(buffer[8] << 8 | buffer[9]);  // Gyro X
    *gy = (int16_t)(buffer[10] << 8 | buffer[11]); // Gyro Y
}

void Send_IMU_Data() {
    int16_t ax, ay, az, gx, gy;
    uint8_t tx_buffer[10];

    MPU6050_ReadAccelGyro(&ax, &ay, &az, &gx, &gy);

    // 16비트 정수를 바이트 배열로 변환 (Little Endian)
    tx_buffer[0] = ax & 0xFF;
    tx_buffer[1] = (ax >> 8) & 0xFF;
    tx_buffer[2] = ay & 0xFF;
    tx_buffer[3] = (ay >> 8) & 0xFF;
    tx_buffer[4] = az & 0xFF;
    tx_buffer[5] = (az >> 8) & 0xFF;
    tx_buffer[6] = gx & 0xFF;
    tx_buffer[7] = (gx >> 8) & 0xFF;
    tx_buffer[8] = gy & 0xFF;
    tx_buffer[9] = (gy >> 8) & 0xFF;

    // UART 전송
    HAL_UART_Transmit(&huart2, tx_buffer, 10, 100);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    while (1) {
        Send_IMU_Data();  // IMU 데이터 전송
        HAL_Delay(100);    // 100ms 마다 전송
    }
}

```
✔ 가속도 X, Y, Z 및 자이로 X, Y 데이터를 읽어 바이트 배열로 변환하여 송신
✔ Little Endian 방식 사용
✔ UART2를 사용하여 PC로 전송 (Baud rate 9600)
---

✅ 2. Python 코드 (PC에서 데이터 수신)


📌 STM32에서 보낸 IMU 데이터를 수신 및 변환

```
import serial
import struct

# 시리얼 포트 설정 (Windows: "COMx", Linux: "/dev/ttyUSBx")
ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)

while True:
    data = ser.read(10)  # 10바이트 수신 (16비트 데이터 5개)

    if len(data) == 10:
        # Little Endian 형식으로 변환
        ax, ay, az, gx, gy = struct.unpack('<hhhhh', data)
        print(f"Accel X: {ax}, Y: {ay}, Z: {az}, Gyro X: {gx}, Y: {gy}")

```
✔ ser.read(10)로 10바이트 수신
✔ struct.unpack('<hhhhh', data)를 사용하여 16비트 정수 5개로 변환
✔ 터미널에 수신된 가속도 및 자이로 데이터 출력
---

✅ 실행 결과

💡 STM32에서 보낸 데이터
데이터
바이트 값 (Little Endian)
ax
0x12 0x34
ay
0x56 0x78
az
0x9A 0xBC
gx
0xDE 0xF0
gy
0x11 0x22
💡 Python에서 수신 및 변환된 출력
```
Accel X: 13330, Y: 30806, Z: -17334, Gyro X: -4130, Y: 8721

```

---

✅ 결론

✔ STM32에서 가속도 X, Y, Z 및 자이로 X, Y 데이터를 바이트 배열로 변환하여 UART로 송신
✔ Python에서 UART 데이터를 읽어 16비트 정수로 변환하여 출력
✔ Little Endian 형식으로 변환하여 일관성 유지
🚀 이제 PC에서 실시간으로 MPU6050의 IMU 데이터를 받을 수 있습니다! 🎯
