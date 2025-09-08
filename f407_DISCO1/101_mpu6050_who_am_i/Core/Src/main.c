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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68 << 1 // MPU6050 I2C address, assuming AD0 is connected to GND (7-bit address << 1 for R/W bit)
#define WHO_AM_I_REG 0x75      // Address of the WHO_AM_I register, used to verify the device identity
#define PWR_MGMT_1   0x6B      // Address of the Power Management 1 register
#define ACCEL_CONFIG 0x1C      // Address of the Accelerometer Configuration register
#define GYRO_CONFIG	 0x1B      // Address of the Gyroscope Configuration register
#define SMPLRT_DIV   0x19      // Address of the Sample Rate Divider register
#define CONFIG       0x1A      // Address of the Configuration register
#define ACCEL_XOUT_H 0x3B      // Address of the high byte of the accelerometer X-axis data
#define GYRO_XOUT_H  0x43      // Address of the high byte of the gyroscope X-axis data

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char f_imu_send = 0;	// Flag to signal that MPU6050 data should be read and sent.
uint8_t who_am_i = 0;	// Variable to store the value of the WHO_AM_I register for verification.
int16_t ax, ay, az;		// 16-bit signed integers to hold accelerometer data for X, Y, and Z axes.
int16_t gx, gy, gz;		// 16-bit signed integers to hold gyroscope data for X, Y, and Z axes.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
// Function to read the MPU6050 device ID.
uint8_t Read_MPU6050_ID();
// Function to write a single byte of data to a specific MPU6050 register.
void MPU6050_Write(uint8_t reg, uint8_t data);
// Function to read multiple bytes of data from a specific MPU6050 register.
void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length);
// Function to initialize the MPU6050 sensor with desired settings.
void MPU6050_Init(void);
// Function to read accelerometer data for all three axes.
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az);
// Function to read gyroscope data for all three axes.
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// This function re-routes the standard `printf` function to use the specified UART peripheral.
// It allows developers to print messages to a connected terminal or serial monitor for debugging.
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
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // Starts the TIM4 timer in interrupt mode, which triggers the `HAL_TIM_PeriodElapsedCallback` function periodically.
  HAL_TIM_Base_Start_IT(&htim4);
  // Calls the MPU6050 initialization function to configure the sensor.
  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(f_imu_send == 1){
		  who_am_i = Read_MPU6050_ID();
		  printf("CHIP ID: %d \r\n", who_am_i);
		  f_imu_send = 0;
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
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// This is the callback function that gets called automatically when a timer period elapses.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // Checks if the interrupt came from TIM4.
    if (htim->Instance == TIM4) {
        // Toggles a pin (PD12) on the board. This can be used as a visual indicator (e.g., an LED) or for debugging.
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        // Sets the flag to 1, signaling the main loop to read and send IMU data.
        f_imu_send = 1;
    }
}

// Function to read the MPU6050's WHO_AM_I register. This is for verifying the device.
// The return value seems to be a placeholder and doesn't return the actual register value.
uint8_t Read_MPU6050_ID() {
	// Reads the WHO_AM_I register using HAL's I2C memory read function.
	uint8_t reg = 0;
	who_am_i =  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);
	return reg;
}

// Wraps the HAL I2C Master Transmit function for a single register write.
void MPU6050_Write(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 2, 100);
}

// Wraps the HAL I2C Master functions for reading multiple bytes from a register.
void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length) {
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, data, length, 100);
}

// Initializes the MPU6050 sensor by writing to its configuration registers.
void MPU6050_Init(void) {
    // 1. Wakes up the sensor by writing 0x00 to the Power Management 1 register.
    MPU6050_Write(PWR_MGMT_1, 0x00);
    HAL_Delay(100);
    // 2. Configures the accelerometer full-scale range to ±2g by writing 0x00.
    MPU6050_Write(ACCEL_CONFIG, 0x00);
    HAL_Delay(10);
    // 2. (commented out) Configures the gyroscope full-scale range to ±250°/s.
    // MPU6050_Write(0x1B, 0x00);
    // HAL_Delay(10);
    // 3. Sets the sample rate divider to 7, resulting in a sample rate of 1kHz / (1+7) = 125Hz.
    MPU6050_Write(SMPLRT_DIV, 0x07);
    HAL_Delay(10);
    // 4. Configures the Digital Low-Pass Filter (DLPF) to a bandwidth of 44Hz.
    MPU6050_Write(CONFIG, 0x03);
    HAL_Delay(10);
}

// Reads raw accelerometer data from the sensor.
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buffer[6]; // Buffer to store 6 bytes of data (2 for each axis).
    // Reads all 6 bytes of accelerometer data, starting from the X-axis high byte register.
    MPU6050_Read(ACCEL_XOUT_H, buffer, 6);
    // Combines the high and low bytes to form 16-bit signed integers for each axis.
    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);
}

// Reads raw gyroscope data from the sensor.
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[6]; // Buffer to store 6 bytes of data (2 for each axis).
    // Reads all 6 bytes of gyroscope data, starting from the X-axis high byte register.
    MPU6050_Read(GYRO_XOUT_H, buffer, 6);
    // Combines the high and low bytes to form 16-bit signed integers for each axis.
    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);
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
