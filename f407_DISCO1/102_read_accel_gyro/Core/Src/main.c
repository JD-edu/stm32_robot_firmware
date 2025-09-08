/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0 // MPU6050 I2C 8-bit address (AD0 pin connected to GND)
#define WHO_AM_I_REG 0x75  // WHO_AM_I register address
#define PWR_MGMT_1    0x6B // Power Management 1 register
#define ACCEL_CONFIG  0x1C // Accelerometer Configuration register
#define GYRO_CONFIG	  0x1B // Gyroscope Configuration register
#define SMPLRT_DIV    0x19 // Sample Rate Divider register
#define CONFIG        0x1A // Configuration register
#define ACCEL_XOUT_H  0x3B  // Accelerometer X-axis data high byte address
#define GYRO_XOUT_H   0x43  // Gyroscope X-axis data high byte address
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char f_imu_read = 0; // Flag to indicate when to read IMU data
char f_uart_send = 0; // Flag to indicate when to send data over UART
uint8_t who_am_i = 0; // Stores the WHO_AM_I register value
int16_t ax, ay, az; // Accelerometer raw data
int16_t gx, gy, gz; // Gyroscope raw data
int16_t pitch, roll, yaw; // Orientation angles (scaled)

// complementary filter variables
float pitch_f = 0.0, roll_f = 0.0, yaw_f = 0.0; // Filtered orientation angles (float)
float alpha = 0.98; // Complementary filter alpha value
float dt = 0.01; // Time step (10ms from TIM2)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Read_MPU6050_ID(); // Reads MPU6050's WHO_AM_I register
void MPU6050_Write(uint8_t reg, uint8_t data); // Writes a single byte to an MPU6050 register
void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length); // Reads multiple bytes from an MPU6050 register
void MPU6050_Init(void); // Initializes the MPU6050 sensor
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az); // Reads accelerometer data
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz); // Reads gyroscope data
void Send_IMU_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* pitch, int16_t* roll, int16_t* yaw); // Sends all IMU data via UART
void Compute_Orientation(); // Computes pitch, roll, and yaw using a complementary filter
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // Starts the timers in interrupt mode
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  // Initializes the MPU6050 sensor
  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Checks for IMU read flag set by TIM2 interrupt
	  if(f_imu_read == 1){
		   MPU6050_ReadAccel(&ax, &ay, &az); // Read accelerometer data
		   MPU6050_ReadGyro(&gx, &gy, &gz);   // Read gyroscope data
		   Compute_Orientation();            // Calculate orientation
		   f_imu_read = 0; // Reset flag
	  }
	  // Checks for UART send flag set by TIM3 interrupt
	  if(f_uart_send == 1){
		  printf("Accel X: %d Y: %d Z: %d Gyro: X: %d, Y: %d, Z: %d \r\n", ax, ay, az, gx, gy, gz);
		  //Send_IMU_Data(&ax, &ay, &az, &gx, &gy, &gz, &pitch, &roll, &yaw);
		  f_uart_send = 0; // Reset flag
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Callback function for TIM2 and TIM3 interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) { // 10ms interrupt for IMU data reading
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        f_imu_read = 1;
    }else if(htim->Instance == TIM3){ // 100ms interrupt for UART data sending
    	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle onboard LED
    	f_uart_send = 1;
    }
}

// Reads the MPU6050 WHO_AM_I register value
uint8_t Read_MPU6050_ID() {
	int uga = 0; // Unused variable, can be removed
	// Read a single byte from the WHO_AM_I register
	who_am_i =  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
	return uga; // Returns an unused variable
}

// Writes a single byte of data to an MPU6050 register
void MPU6050_Write(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data}; // Buffer containing register address and data
    if(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 2, 100) != HAL_OK){
    	printf("I2C error\n");
    }
}

// Reads multiple bytes from a specified MPU6050 register
void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length) {
    // Transmits the register address to start reading from
	HAL_StatusTypeDef status;

	    // 1. 레지스터 주소 전송
	    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100);
	    if (status != HAL_OK) {
	        // 전송 에러 종류 확인
	        switch (status) {
	            case HAL_ERROR:
	                printf("I2C Transmit Error: HAL_ERROR\r\n");
	                break;
	            case HAL_BUSY:
	                printf("I2C Transmit Error: HAL_BUSY\r\n");
	                break;
	            case HAL_TIMEOUT:
	                printf("I2C Transmit Error: HAL_TIMEOUT\r\n");
	                break;
	            default:
	                printf("I2C Transmit Error: Unknown\r\n");
	                break;
	        }
	        return; // 에러 발생 시 함수 종료
	    }

	    // 2. 데이터 수신
	    status = HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, data, length, 100);
	    if (status != HAL_OK) {
	        // 수신 에러 종류 확인
	        switch (status) {
	            case HAL_ERROR:
	                printf("I2C Receive Error: HAL_ERROR\r\n");
	                break;
	            case HAL_BUSY:
	                printf("I2C Receive Error: HAL_BUSY\r\n");
	                break;
	            case HAL_TIMEOUT:
	                printf("I2C Receive Error: HAL_TIMEOUT\r\n");
	                break;
	            default:
	                printf("I2C Receive Error: Unknown\r\n");
	                break;
	        }
	        return; // 에러 발생 시 함수 종료
	    }
}

// Initializes the MPU6050 sensor with a few key settings
void MPU6050_Init(void) {
    // 1. Wake up the sensor (Power Management 1)
    MPU6050_Write(PWR_MGMT_1, 0x00);
    HAL_Delay(100);

    // 2. Set accelerometer range to ±2g
    MPU6050_Write(ACCEL_CONFIG, 0x00);
    HAL_Delay(10);

    // 2. Set gyroscope range to ±250°/s
    //MPU6050_Write(0x1B, 0x00);
    //HAL_Delay(10);

    // 3. Set sample rate to 125Hz (1kHz / (1+7))
    MPU6050_Write(SMPLRT_DIV, 0x07);
    HAL_Delay(10);

    // 4. Set Digital Low-Pass Filter (DLPF) to 44Hz
    MPU6050_Write(CONFIG, 0x03);
    HAL_Delay(10);
}

// Reads and combines high and low bytes of accelerometer data
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buffer[6];  // Buffer for 6 bytes of accelerometer data (X, Y, Z)

    MPU6050_Read(ACCEL_XOUT_H, buffer, 6);

    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);  // Combine X high and low bytes
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);  // Combine Y high and low bytes
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);  // Combine Z high and low bytes
}

// Reads and combines high and low bytes of gyroscope data
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[6];  // Buffer for 6 bytes of gyroscope data (X, Y, Z)

    MPU6050_Read(GYRO_XOUT_H, buffer, 6);

    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);  // Combine X high and low bytes
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);  // Combine Y high and low bytes
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);  // Combine Z high and low bytes
}

// Packages all IMU data into a buffer and sends it via UART
void Send_IMU_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* pitch, int16_t* roll, int16_t* yaw) {

    uint8_t tx_buffer[18];

    // Converts 16-bit integer data into a byte array for transmission (Little Endian)
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
    tx_buffer[12] = *pitch & 0xFF;
    tx_buffer[13] = (*pitch >> 8) & 0xFF;
    tx_buffer[14] = *roll & 0xFF;
    tx_buffer[15] = (*roll >> 8) & 0xFF;
    tx_buffer[16] = *yaw & 0xFF;
    tx_buffer[17] = (*yaw >> 8) & 0xFF;

    // Transmits the data buffer over UART
    HAL_UART_Transmit(&huart2, tx_buffer, 18, 100);
}

// Computes orientation angles using a complementary filter
void Compute_Orientation() {
    // Calculate pitch and roll from accelerometer data
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958; // Convert radians to degrees
    float acc_roll  = atan2(ay, az) * 57.2958;

    // Convert gyroscope raw data to degrees per second
    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;
    float gz_dps = gz / 131.0;

    // Apply the complementary filter to combine accelerometer and gyroscope data
    pitch_f = alpha * (pitch_f + gy_dps * dt) + (1 - alpha) * acc_pitch;
    roll_f = alpha * (roll_f + gx_dps * dt) + (1 - alpha) * acc_roll;
    yaw_f += gz_dps * dt; // Yaw is only integrated from gyroscope

    // Convert float angles to integer angles for storage
    pitch = (int16_t)(pitch_f * 100);
    roll = (int16_t)(roll_f * 100);
    yaw = (int16_t)(yaw_f * 100);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
