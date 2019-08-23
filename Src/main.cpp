/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_entry.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <SparkFunMPU9250-DMP.h>
extern "C" {
#include "stm32_mpu9250_spi.h"
#include "stm32_utils.h"
#include "app_conf.h"
//#include "utilities_common.h"
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
	uint8_t			number;
	GPIO_TypeDef 	*port;
	uint16_t 		pin;
} SpiSlaveHandler_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

SpiSlaveHandler_t spiSlavesArray[] =
{
	[0] = {.number = 0, .port = SPI1_CS_0_GPIO_Port, .pin = SPI1_CS_0_Pin},
	[1] = {.number = 1, .port = SPI1_CS_1_GPIO_Port, .pin = SPI1_CS_1_Pin},
	[2] = {.number = 2, .port = SPI1_CS_2_GPIO_Port, .pin = SPI1_CS_2_Pin}
};
#define NUMBER_OF_SENSORS (sizeof(spiSlavesArray)/sizeof(SpiSlaveHandler_t))
volatile uint8_t isMpuMeasureReady = 0u;

volatile uint8_t mpuDataToBeSend[75];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_RF_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

MPU9250_DMP IMUs[NUMBER_OF_SENSORS];
void printIMUData(uint8_t sensor_number);
void readMpuDataCallback(void);
void SetupMPUSensors(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool test_whoAmI();
#include "scheduler.h"

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_RF_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	///////////////// SPI FOR MPU9250 SET ///////////////////////
	set_spi_handler(&hspi1);

	///////////////// SENSOR CHECK ///////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
	  set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);
	  if(!test_whoAmI())
		  while(1)
		  {
			  // Sensor check fail
			  printf("Sensor check fail! Try again.\n");
			  delay_ms(1000);
		  }
	}
	printf("Sensors check passed.\n");

	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);
		IMUs[i].resetDevice();
	}
	printf("Sensors reset.\n");

	///// FILL MOCK DATA ///////
	mpuDataToBeSend[0] = 'S';
	mpuDataToBeSend[1] = 'T';
	mpuDataToBeSend[2] = 'T';

	for(unsigned i = 0; i < (75 - 3) / 18; i++)
	{
		for(unsigned k = 0; k < 6; k++)
			mpuDataToBeSend[3 + i * 18 + k] = 'A';
		for(unsigned k = 0; k < 6; k++)
			mpuDataToBeSend[3 + i * 18 + k + 6] = 'G';
		for(unsigned k = 0; k < 6; k++)
			mpuDataToBeSend[3 + i * 18 + k + 12] = 'M';
	}



  ////////////////// MEASUREMENT START //////////////////////
  printf("Press SW1 to start.\n");
  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
  {
	  delay_ms(20);
  }
  printf("Measurement start.\n");

  ////////////////// SENSORS SETUP //////////////////////////
  SetupMPUSensors();

#if MPU_DMP_DATA_ENABLE
  ////////////////// RESET FIFO /////////////////////////////
  for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
  {
	set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);
	IMUs[i].resetFifo();
  }
  printf("Fifo reset done.\n");
#endif

  ////////////////// RESET SYSTICK /////////////////////////
//  __disable_irq();
//  uwTick = 0lu;
//  __enable_irq();

  ////////////////// SET MPU9250 INTERRUPT /////////////////////////
  SCH_RegTask( CFG_TASK_MPU9250_INT_ID, readMpuDataCallback );
  set_CS_portpin(spiSlavesArray[0].port, spiSlavesArray[0].pin);
  IMUs[0].enableInterrupt(1);
  uint32_t primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();          /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  isMpuMeasureReady = 1u;
  __set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/
  ////////////////// LOOP START ///////////////////////////
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Run all tasks
	SCH_Run(SCH_DEFAULT);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_0_Pin|SPI1_CS_1_Pin|SPI1_CS_2_Pin|SPI1_CS_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_0_Pin SPI1_CS_1_Pin SPI1_CS_2_Pin SPI1_CS_3_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_0_Pin|SPI1_CS_1_Pin|SPI1_CS_2_Pin|SPI1_CS_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void SetupMPUSensors(void)
{
	////////////////// SETUP /////////////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);

		// Call IMUs[i].begin() to verify communication and initialize
		if (IMUs[i].begin() != INV_SUCCESS)
		{
			while (1)
			{
				printf("Unable to communicate with MPU-9250\n");
				printf("Check connections, and try again.\n");
				HAL_Delay(5000);
			}
		}

#if MPU_DMP_DATA_ENABLE
		// DMP_FEATURE_LP_QUAT can also be used. It uses the
		// accelerometer in low-power mode to estimate quat's.
		// DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
		/*(	DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
			DMP_FEATURE_GYRO_CAL, // Use gyro calibration
			MPU_SAMPLE_RATE) // Set DMP FIFO rate to 200 Hz */
		if ( IMUs[i].dmpBegin( DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, MPU_SAMPLE_RATE) != INV_SUCCESS )
			while (1)
			{
				printf("Unable to setup DMP in MPU-9250 nr %d\n", i);
				printf("Check connections, and try again.\n");
				HAL_Delay(1000);
			}
#else
		// Enable all sensors, and set sample rates to 4Hz.
		// (Slow so we can see the interrupt work.)
		IMUs[i].setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
		IMUs[i].setSampleRate(MPU_SAMPLE_RATE); // Set accel/gyro sample rate to 100 Hz
		IMUs[i].setCompassSampleRate(MPU_SAMPLE_RATE); // Set mag rate to 100 Hz

		if(i == 0)
		{
			// Use enableInterrupt() to configure the MPU-9250's
			// interrupt output as a "data ready" indicator.
	//		IMUs[i].enableInterrupt(1); Not yet

			// The interrupt level can either be active-high or low.
			// Configure as active-low, since we'll be using the pin's
			// internal pull-up resistor.
			// Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
			IMUs[i].setIntLevel(INT_ACTIVE_LOW);

			// The interrupt can be set to latch until data has
			// been read, or to work as a 50us pulse.
			// Use latching method -- we'll read from the sensor
			// as soon as we see the pin go LOW.
			// Options are INT_LATCHED or INT_50US_PULSE
			IMUs[i].setIntLatched(INT_50US_PULSE);
		}
#endif
	}

	////////////////// Disable interrupts /////////////////////////////
	set_CS_portpin(spiSlavesArray[0].port, spiSlavesArray[0].pin);
	if ( IMUs[0].enableInterrupt(0) != INV_SUCCESS)
	while (1)
	{
		printf("Unable to disable interrupt.\n");
		printf("Check connections, and try again.\n");
		HAL_Delay(1000);
	}

	printf("Setup done.\n");
}

void printIMUData(uint8_t sensor_number)
{
#if MPU_DMP_DATA_ENABLE
	char str1[10], str2[10];
	// After calling dmpUpdateFifo() the ax, gx, mx, etc. values
	// are all updated.
	// Quaternion values are, by default, stored in Q30 long
	// format. calcQuat turns them into a float between -1 and 1
	float q0 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qw);
	float q1 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qx);
	float q2 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qy);
	float q3 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qz);

	// Quaternions
	ftoa(q0, str1);
	ftoa(q1, str2);
	printf("Sensor: %d\n", sensor_number);
	printf("Q: %s, %s", str1, str2);
	ftoa(q2, str1);
	ftoa(q3, str2);
	printf(", %s, %s\n", str1, str2);

	// Euler angles
//	ftoa(IMUs[sensor_number].roll, str1);
//	ftoa(IMUs[sensor_number].pitch, str2);
//	printf("R/P/Y: %s, %s", str1, str2);
//	ftoa(IMUs[sensor_number].yaw, str1);
//	printf(", %s\n", str1);
#else
	char str1[10], str2[10], str3[10];
	// After calling update() the ax, ay, az, gx, gy, gz, mx,
	// my, mz, time, and/or temerature class variables are all
	// updated. Access them by placing the object. in front:

	printf("Sensor: %d\n", sensor_number);

	// Use the calcAccel, calcGyro, and calcMag functions to
	// convert the raw sensor readings (signed 16-bit values)
	// to their respective units.
	float data1 = IMUs[sensor_number].calcAccel(IMUs[sensor_number].ax);
	float data2 = IMUs[sensor_number].calcAccel(IMUs[sensor_number].ay);
	float data3 = IMUs[sensor_number].calcAccel(IMUs[sensor_number].az);

	// Accel
	ftoa(data1, str1);
	ftoa(data2, str2);
	ftoa(data3, str3);
	printf("A: %s, %s, %s g\n", str1, str2, str3);

	// Gyro
	data1 = IMUs[sensor_number].calcGyro(IMUs[sensor_number].gx);
	data2 = IMUs[sensor_number].calcGyro(IMUs[sensor_number].gy);
	data3 = IMUs[sensor_number].calcGyro(IMUs[sensor_number].gz);
	ftoa(data1, str1);
	ftoa(data2, str2);
	ftoa(data3, str3);
	printf("G: %s, %s, %s dps\n", str1, str2, str3);

	// Mag
	data1 = IMUs[sensor_number].calcMag(IMUs[sensor_number].mx);
	data2 = IMUs[sensor_number].calcMag(IMUs[sensor_number].my);
	data3 = IMUs[sensor_number].calcMag(IMUs[sensor_number].mz);
	ftoa(data1, str1);
	ftoa(data2, str2);
	ftoa(data3, str3);
	printf("M: %s, %s, %s uT\n", str1, str2, str3);
#endif

	// Time
	printf("Time: %lu ms\n", IMUs[sensor_number].time);
}

extern "C" {
#include "stm32_mpu9250_spi.h"
}
bool test_whoAmI()
{
	uint8_t readData[5] = {0};
	if(spi_read(0u, 0x75u, 4u, readData)) {
		printf("Check error\n");
		return false;
	}
	else if(readData[0] == 0x71) {
		return true;
	}
	else {
		printf("ID matching error\n");
		return false;
	}
}

void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
	  // Run all tasks
	  	SCH_Run(SCH_DEFAULT);
  }
}

void readMpuDataCallback(void)
{
	uint32_t primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
	__disable_irq();          /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
	if(isMpuMeasureReady)
	{
		isMpuMeasureReady = 0u;
		__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/

		static uint8_t numOfIters = 0u;
		static constexpr uint8_t numOfItersToPrint = MPU_SAMPLE_RATE; // value set for 1Hz printf refresh rate

		// Get timestamp
		unsigned long timestamp;
	    get_ms(&timestamp);
	    for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	    {
	    	IMUs[i].time = timestamp;
	    }

		for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
		{
			set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);

#if MPU_DMP_DATA_ENABLE
			// Check for new data in the FIFO
			if (i == 0 )
				while(!IMUs[i].fifoAvailable())
				{
					delay_ms(1);
				}

			// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
			if ( IMUs[i].dmpUpdateFifo() == INV_SUCCESS)
			{
				// computeEulerAngles can be used -- after updating the
				// quaternion values -- to estimate roll, pitch, and yaw
	//			IMUs[i].computeEulerAngles();
			}
			else
			{
				printf("DMP update fifo read error!\n");
			}
#else
			// Check whether magnetometer data is ready
			if(i == 0)
			{
//				if((numOfIters % 5) == 0)
					SCH_SetTask(1<<CFG_TASK_MPU_DATA_READY_ID, CFG_SCH_PRIO_1);

				delay_ms(1);
			}

		    // Call update() to update the imu objects sensor data.
//			if(IMUs[i].update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS) != INV_SUCCESS)
			if(IMUs[i].allDataUpdate() != INV_SUCCESS)
				printf("IMU data read error!\n");
#endif
		}

		numOfIters++;

		if(numOfIters >= numOfItersToPrint) {
			for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
			{
//				printIMUData(i);
				printf("Sens: %d, A, G, M, T\n", i);
			}

			HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
			numOfIters = 0u;
			printf("\n");
		}

		primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
		__disable_irq();          /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
		isMpuMeasureReady = 1u;
		__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/
	}
	else
	{
		__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
