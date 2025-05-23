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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68 << 1 // AD0 ?? GND 기�? (7비트 주소 << 1)
#define WHO_AM_I_REG 0x75  // WHO_AM_I ?���???????��?�� 주소
#define PWR_MGMT_1    0x6B
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG	  0x1B
#define SMPLRT_DIV    0x19
#define CONFIG        0x1A
#define ACCEL_XOUT_H  0x3B  // �?????��?�� X�???? ?��?�� 바이?�� 주소
#define GYRO_XOUT_H   0x43
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
char f_imu_read = 0;
char f_uart_send = 0;
uint8_t who_am_i = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t pitch, roll, yaw;

// complementary filter variables
float pitch_f = 0.0, roll_f = 0.0, yaw_f = 0.0;
float alpha = 0.98;
float dt = 0.01;
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
uint8_t Read_MPU6050_ID();
void MPU6050_Write(uint8_t reg, uint8_t data);
void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length);
void MPU6050_Init(void);
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az);
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz);
void Send_IMU_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* pitch, int16_t* roll, int16_t* yaw) ;
void Compute_Orientation();
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
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(f_imu_read == 1){
		   MPU6050_ReadAccel(&ax, &ay, &az);
		   MPU6050_ReadGyro(&gx, &gy, &gz);
		   Compute_Orientation();
		   //printf("Accel X: %d Y: %d Z: %d Gyro: X: %d, Y: %d, Z: %d \r\n", ax, ay, az, gx, gy, gz);
		   //Send_IMU_Data(&ax, &ay, &az, &gx, &gy, &gz);
		   f_imu_read = 0;
	  }
	  if(f_uart_send == 1){
		  printf("Accel X: %d Y: %d Z: %d Gyro: X: %d, Y: %d, Z: %d \r\n", ax, ay, az, gx, gy, gz);
		  //Send_IMU_Data(&ax, &ay, &az, &gx, &gy, &gz, &pitch, &roll, &yaw);
		  f_uart_send = 0;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
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
  hi2c1.Init.ClockSpeed = 50000;
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
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  htim3.Init.Prescaler = 8400-1;
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
  huart2.Init.BaudRate = 115200;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
    	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        f_imu_read = 1;
    }else if(htim->Instance == TIM3){
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    	f_uart_send = 1;
    }
}

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
    // 1️⃣ ?��?�� 모드 ?��?�� (PWR_MGMT_1)
    MPU6050_Write(PWR_MGMT_1, 0x00);
    HAL_Delay(100);

    // 2️⃣ �?????��?�� 범위 ?��?�� (±2g)
    MPU6050_Write(ACCEL_CONFIG, 0x00);  // 00 = ±2g
    HAL_Delay(10);

    // 2️⃣ ?��?���??? 범위 ?��?�� (±250°/s)
    //MPU6050_Write(0x1B, 0x00);  // 00 = ±250°/s
    //HAL_Delay(10);

    // 3️⃣ ?��?���???? ?��?�� ?��?�� (SMPLRT_DIV)
    MPU6050_Write(SMPLRT_DIV, 0x07);  // 1kHz / (1 + 7) = 125Hz ?��?���????
    HAL_Delay(10);

    // 4️⃣ ???�� ?���???? ?��?�� ?��?�� (CONFIG)
    MPU6050_Write(CONFIG, 0x03);  // 44Hz ?��?�� ?��?��
    HAL_Delay(10);
}

void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buffer[6];  // �?????��?�� ?��?��?�� 6바이?�� (X, Y, Z)

    MPU6050_Read(ACCEL_XOUT_H, buffer, 6);

    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);  // X�????
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);  // Y�????
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);  // Z�????
}

void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[6];  // ?��?���??? ?��?��?�� 6바이?�� (X, Y, Z)

    MPU6050_Read(GYRO_XOUT_H, buffer, 6);

    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);  // X�???
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);  // Y�???
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);  // Z�???
}

void Send_IMU_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* pitch, int16_t* roll, int16_t* yaw) {

    uint8_t tx_buffer[18];

    // 16비트 ?��?���??? 바이?�� 배열�??? �????�� (Little Endian)
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

    // UART ?��?��
    HAL_UART_Transmit(&huart2, tx_buffer, 18, 100);
}

void Compute_Orientation() {
    // 가속도계 기반 pitch, roll 계산
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958; // rad to deg
    float acc_roll  = atan2(ay, az) * 57.2958;

    // 자이로스코프 데이터를 각속도(°/s)로 변환 (MPU6050 기본 감도: 250dps)
    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;
    float gz_dps = gz / 131.0;

    // 상보 필터 적용
    pitch_f = alpha * (pitch_f + gy_dps * dt) + (1 - alpha) * acc_pitch;
    roll_f = alpha * (roll_f + gx_dps * dt) + (1 - alpha) * acc_roll;
    yaw_f += gz_dps * dt; // Yaw는 자이로만 사용

    // float 값을 int16_t로 변환
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

#ifdef  USE_FULL_ASSERT
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
