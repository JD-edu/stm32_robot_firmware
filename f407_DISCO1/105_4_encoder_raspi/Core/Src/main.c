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
#define WHO_AM_I_REG 0x75  // WHO_AM_I ?���???????????????��?�� 주소
#define PWR_MGMT_1    0x6B
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG	  0x1B
#define SMPLRT_DIV    0x19
#define CONFIG        0x1A
#define ACCEL_XOUT_H  0x3B  // �?????????????��?�� X�???????????? ?��?�� 바이?�� 주소
#define GYRO_XOUT_H   0x43

// protocols
#define HEAD 		0xf5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char f_imu_read = 0;
char f_uart_send = 0;
uint8_t who_am_i = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t pitch, roll, yaw;
// Serial data buffer
uint8_t rxData[1];
const float dt = 0.025;
typedef struct {
    float angle;  // 추정?�� 각도
    float bias;   // ?��?��로스코프 바이?��?��
    float rate;   // ?��?��로스코프 측정�????????

    float P[2][2];  // ?���???????? 공분?�� ?��?��
    float K[2];     // 칼만 ?��?��
    float y;        // 측정 차이
    float S;        // ?���???????? 공분?��
} KalmanFilter;
static int16_t encoder1 = 0;  // ?��코더 �?? ???�� �???��
static int16_t encoder2 = 0;
static int16_t encoder3 = 0;
static int16_t encoder4 = 0;

KalmanFilter kalman_pitch, kalman_roll, kalman_yaw;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Read_MPU6050_ID();
void MPU6050_Write(uint8_t reg, uint8_t data);
void MPU6050_Read(uint8_t reg, uint8_t* data, uint8_t length);
void MPU6050_Init(void);
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az);
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz);
void Send_IMU_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* pitch, int16_t* roll, int16_t* yaw) ;
void Kalman_Init(KalmanFilter *kf);
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  MPU6050_Init();
  Kalman_Init(&kalman_pitch);
  Kalman_Init(&kalman_roll);
  Kalman_Init(&kalman_yaw);

  HAL_UART_Receive_IT(&huart2, rxData, 1); // ?��?��?��?�� 기반 ?��?�� ?��?��
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // ?��코더 ?��?��?��?��?�� ?��?��
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(f_imu_read == 1){
		  MPU6050_ReadAccel(&ax, &ay, &az);
		  MPU6050_ReadGyro(&gx, &gy, &gz);
		  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		  Compute_Orientation_Kalman();
		  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		  //printf("Accel X: %d Y: %d Z: %d Gyro: X: %d, Y: %d, Z: %d \r\n", ax, ay, az, gx, gy, gz);
		  //Send_IMU_Data(&ax, &ay, &az, &gx, &gy, &gz);
		  f_imu_read = 0;
	  }
	  if(f_uart_send == 1){
		  //printf("Pitch %d Roll %d Yaw %d \r\n", pitch, roll, yaw);
		  Send_IMU_Data(&ax, &ay, &az, &gx, &gy, &gz, &pitch, &roll, &yaw);
		  encoder1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
		  encoder2++;
		  encoder3++;
		  encoder4++;
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
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
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
  htim2.Init.Period = 250-1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE13 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        f_imu_read = 1;
    }else if(htim->Instance == TIM3){
    	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
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
	// MPU6050 software reset
	 MPU6050_Write(PWR_MGMT_1, 0x80);
	 HAL_Delay(100);

    // 1️⃣ ?��?�� 모드 ?��?�� (PWR_MGMT_1)
    MPU6050_Write(PWR_MGMT_1, 0x00);
    HAL_Delay(100);

    // 2️⃣ �???????????��?�� 범위 ?��?�� (±2g)
    MPU6050_Write(ACCEL_CONFIG, 0x00);  // 00 = ±2g
    HAL_Delay(10);

    // 2️⃣ ?��?���????????? 범위 ?��?�� (±250°/s)
    MPU6050_Write(0x1B, 0x00);  // 00 = ±250°/s
    HAL_Delay(10);

    // 3️⃣ ?��?���?????????? ?��?�� ?��?�� (SMPLRT_DIV)
    MPU6050_Write(SMPLRT_DIV, 0x07);  // 1kHz / (1 + 7) = 125Hz ?��?���??????????
    HAL_Delay(10);

    // 4️⃣ ???�� ?���?????????? ?��?�� ?��?�� (CONFIG)
    MPU6050_Write(CONFIG, 0x03);  // 44Hz ?��?�� ?��?��
    HAL_Delay(10);
}

void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buffer[6];  // �???????????��?�� ?��?��?�� 6바이?�� (X, Y, Z)

    MPU6050_Read(ACCEL_XOUT_H, buffer, 6);

    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);  // X�??????????
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);  // Y�??????????
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);  // Z�??????????
}

void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[6];  // ?��?���????????? ?��?��?�� 6바이?�� (X, Y, Z)

    MPU6050_Read(GYRO_XOUT_H, buffer, 6);

    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);  // X�?????????
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);  // Y�?????????
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);  // Z�?????????
}

void Send_IMU_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* pitch, int16_t* roll, int16_t* yaw) {

    uint8_t tx_buffer[29];

    // 16비트 ?��?���????????? 바이?�� 배열�????????? �??????????�� (Little Endian)
    tx_buffer[0] = 0xf5;
    tx_buffer[1] = 26;
    tx_buffer[2] = *ax & 0xFF;
    tx_buffer[3] = (*ax >> 8) & 0xFF;
    tx_buffer[4] = *ay & 0xFF;
    tx_buffer[5] = (*ay >> 8) & 0xFF;
    tx_buffer[6] = *az & 0xFF;
    tx_buffer[7] = (*az >> 8) & 0xFF;
    tx_buffer[8] = *gx & 0xFF;
    tx_buffer[9] = (*gx >> 8) & 0xFF;
    tx_buffer[10] = *gy & 0xFF;
    tx_buffer[11] = (*gy >> 8) & 0xFF;
    tx_buffer[12] = *gz & 0xFF;
    tx_buffer[13] = (*gz >> 8) & 0xFF;
    tx_buffer[14] = *pitch & 0xFF;
    tx_buffer[15] = (*pitch >> 8) & 0xFF;
    tx_buffer[16] = *roll & 0xFF;
    tx_buffer[17] = (*roll >> 8) & 0xFF;
    tx_buffer[18] = *yaw & 0xFF;
    tx_buffer[19] = (*yaw >> 8) & 0xFF;
    tx_buffer[20] = (encoder1 >> 8) & 0xff;
    tx_buffer[21] = encoder1 & 0xff;
    tx_buffer[22] = (encoder2 >> 8) & 0xff;
    tx_buffer[23] = encoder2 & 0xff;
    tx_buffer[24] = (encoder3 >> 8) & 0xff;
    tx_buffer[25] = encoder3 & 0xff;
    tx_buffer[26] = (encoder4 >> 8) & 0xff;
    tx_buffer[27] = encoder4 & 0xff;
    tx_buffer[28] = 0x00;


    // UART ?��?��
    HAL_UART_Transmit(&huart2, tx_buffer, 29, 100);
}

void Kalman_Init(KalmanFilter *kf) {
    kf->angle = 0.0;
    kf->bias = 0.0;
    kf->P[0][0] = 1.0;  kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;  kf->P[1][1] = 1.0;
}

float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt) {
    // 1️⃣ ?���?????? ?���??????
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // ?���?????? 공분?�� ?��?�� ?��?��?��?��
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + 0.001);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += 0.003;

    // 2️⃣ 보정 ?���??????
    kf->S = kf->P[0][0] + 0.03;
    kf->K[0] = kf->P[0][0] / kf->S;
    kf->K[1] = kf->P[1][0] / kf->S;

    kf->y = newAngle - kf->angle;
    kf->angle += kf->K[0] * kf->y;
    kf->bias += kf->K[1] * kf->y;

    // ?���?????? 공분?�� ?��?��?��?��
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= kf->K[0] * P00_temp;
    kf->P[0][1] -= kf->K[0] * P01_temp;
    kf->P[1][0] -= kf->K[1] * P00_temp;
    kf->P[1][1] -= kf->K[1] * P01_temp;

    return kf->angle;
}

void Compute_Orientation_Kalman() {
    // �???????��?���?????? 기반 pitch, roll 계산
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958;
    float acc_roll  = atan2(ay, az) * 57.2958;

    // ?��?��로스코프 값을 deg/s ?��?���?????? �???????��
    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;
    float gz_dps = gz / 131.0;

    // 칼만 ?��?�� ?��?��
    float pitch_f = Kalman_Update(&kalman_pitch, acc_pitch, gy_dps, dt);
    float roll_f  = Kalman_Update(&kalman_roll, acc_roll, gx_dps, dt);
    float yaw_f   = Kalman_Update(&kalman_yaw, yaw_f, gz_dps, dt);

    // float -> int �???????��
    pitch = (int16_t)(pitch_f * 100);
    roll  = (int16_t)(roll_f * 100);
    yaw   = (int16_t)(yaw_f * 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if(rxData[0] == 'a'){
        	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
        	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
        }else if(rxData[0] == 'b'){
        	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
        	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
        }
    }
    HAL_UART_Receive_IT(&huart2, rxData, 1); // ?��?�� ?��?�� ?��?��
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
