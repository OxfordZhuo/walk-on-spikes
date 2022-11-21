/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId buttonTaskHandle;
osThreadId transmitTaskHandle;
osThreadId readTaskHandle;
osThreadId killTaskHandle;
/* USER CODE BEGIN PV */
#define buffer_size	1000
int32_t pre_Buffer[buffer_size];
int32_t aft_Buffer[buffer_size];
uint8_t first_half_done = 0;
uint8_t second_half_done = 0;
uint8_t start_recording = 0;
uint8_t volume_height = 0;
uint32_t start_status = 0;
int32_t buffmax = 0;
int32_t buffmin = 0;
int32_t mean = 0;
int32_t sum = 0;
uint32_t i;
uint32_t j;
//uint32_t data_index = 0;
uint32_t puttonPushed = 0;
//uint32_t killCounter = 0;
uint32_t resetCounter = 0;
uint32_t blank_count = 0;
HAL_StatusTypeDef UART_status;
//int16_t accelero[3];
//float gyro[3];
//int16_t magneto[3];
//float hsensor;
char str[220];
char row1[20];
char row2[20];
char row3[20];
char row4[20];
char row5[20];
char row6[20];
char row7[20];
char row8[20];
char row9[20];
char row10[20];
char row11[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
void StartButtonTask(void const * argument);
void StartTransmitTask(void const * argument);
void StartReadTask(void const * argument);
void StartKillTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of buttonTask */
  osThreadDef(buttonTask, StartButtonTask, osPriorityNormal, 0, 128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);

  /* definition and creation of transmitTask */
  osThreadDef(transmitTask, StartTransmitTask, osPriorityNormal, 0, 128);
  transmitTaskHandle = osThreadCreate(osThread(transmitTask), NULL);

  /* definition and creation of readTask */
  osThreadDef(readTask, StartReadTask, osPriorityNormal, 0, 128);
  readTaskHandle = osThreadCreate(osThread(readTask), NULL);

  /* definition and creation of killTask */
  osThreadDef(killTask, StartKillTask, osPriorityNormal, 0, 128);
  killTaskHandle = osThreadCreate(osThread(killTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	    HAL_Delay(100);
//		BSP_ACCELERO_AccGetXYZ(accelero);
//		BSP_GYRO_GetXYZ(gyro);
//		BSP_MAGNETO_GetXYZ(magneto);
//		hsensor = BSP_HSENSOR_ReadHumidity();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 250;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB_Pin */
  GPIO_InitStruct.Pin = PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
	first_half_done = 1;
}
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
	second_half_done = 1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	puttonPushed = 1;

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartButtonTask */
/**
  * @brief  Function implementing the buttonTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    //ITM_Port32(31) = 1;
    if (puttonPushed==1){
    	puttonPushed = 0;
    	resetCounter = 1;//this indicates the start of the game
    	//initialize 10 rows
        sprintf(row1, "\r*|          |*\n");
        sprintf(row2, "\r*|          |*\n");
        sprintf(row3, "\r*|          |*\n");
        sprintf(row4, "\r*|          |*\n");
        sprintf(row5, "\r*|          |*\n");
        sprintf(row6, "\r*|          |*\n");
        sprintf(row7, "\r*|          |*\n");
        sprintf(row8, "\r*|          |*\n");
        sprintf(row9, "\r*|          |*\n");
        sprintf(row10, "\r*|         O|*\n");
        sprintf(row11, "\r**************\n");
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTransmitTask */
/**
* @brief Function implementing the transmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmitTask */
void StartTransmitTask(void const * argument)
{
  /* USER CODE BEGIN StartTransmitTask */
	  HAL_UART_Init(&huart1);
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    while(resetCounter ==0){
    //do nothing until the button is pressed
    }
    //for each row, copy the last row's content
    for(uint16_t copyIndex =0;copyIndex<20;copyIndex++){
    row10[copyIndex] = row9[copyIndex];
    row9[copyIndex] = row8[copyIndex];
    row8[copyIndex] = row7[copyIndex];
    row7[copyIndex] = row6[copyIndex];
    row6[copyIndex] = row5[copyIndex];
    row5[copyIndex] = row4[copyIndex];
    row4[copyIndex] = row3[copyIndex];
    row3[copyIndex] = row2[copyIndex];
    row2[copyIndex] = row1[copyIndex];
    }
    if (row10[12-volume_height]=='_'){
    	row10[12-volume_height] = 'X';
    	resetCounter = 0;
    }
    else{
    	row10[12-volume_height] = 'O';
    }
    //generate the row1
    if (blank_count == 0){ //when blank_count=0, we generate spikes, otherwise we leave it blank
    	blank_count +=1;
    	uint8_t first_spike =  rand()%7;//generate a random number between 0-6
    	//generate different spikes
    	if (first_spike == 0){
    		sprintf(row1, "\r*|    ______|*\n");
    	}
    	else if(first_spike == 1){
    		sprintf(row1, "\r*|_    _____|*\n");
    	}
    	else if(first_spike == 2){
    		sprintf(row1, "\r*|__    ____|*\n");
    	}
    	else if(first_spike == 3){
    		sprintf(row1, "\r*|___    ___|*\n");
    	}
    	else if(first_spike == 4){
    		sprintf(row1, "\r*|____    __|*\n");
    	}
    	else if(first_spike == 5){
    		sprintf(row1, "\r*|_____    _|*\n");
    	}
    	else if(first_spike == 6){
    		sprintf(row1, "\r*|______    |*\n");
    	}
    }
    else if(blank_count == 1){//leave blank(no spikes)
    	blank_count +=1;
    	sprintf(row1, "\r*|          |*\n");
    }
    else{//leave blank(no spikes)
    	blank_count = 0;
    	sprintf(row1, "\r*|          |*\n");
    }
//    HAL_UART_Transmit(&huart1, (uint8_t*) row1, (uint16_t) strlen(row1), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row2, (uint16_t) strlen(row2), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row3, (uint16_t) strlen(row3), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row4, (uint16_t) strlen(row4), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row5, (uint16_t) strlen(row5), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row6, (uint16_t) strlen(row6), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row7, (uint16_t) strlen(row7), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row8, (uint16_t) strlen(row8), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row9, (uint16_t) strlen(row9), 10000);
//    HAL_UART_Transmit(&huart1, (uint8_t*) row10, (uint16_t) strlen(row10), 10000);
    sprintf(str, "%s%s%s%s%s%s%s%s%s%s%s", row1, row2, row3, row4, row5, row6, row7, row8, row9, row10, row11);
    HAL_UART_Transmit(&huart1, (uint8_t*) str, (uint16_t) strlen(str), 10000);
  }
  /* USER CODE END StartTransmitTask */
}

/* USER CODE BEGIN Header_StartReadTask */
/**
* @brief Function implementing the readTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadTask */
void StartReadTask(void const * argument)
{
  /* USER CODE BEGIN StartReadTask */

  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    while(resetCounter ==0){
    //do nothing until the button is pressed
    }
    if(start_recording==0){
    HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, pre_Buffer, buffer_size);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    start_recording=1;
    sum = 0;
    }
	  if (first_half_done == 1){
		  for(i = 0; i<buffer_size/2; i++){
			  aft_Buffer[i] = pre_Buffer[i]>>20;
			  if (aft_Buffer[i]>buffmax){
				  buffmax = aft_Buffer[i];
			  }
			  if (aft_Buffer[i]<buffmin){
				  buffmin = aft_Buffer[i];
			  }
			  if(aft_Buffer[i]<0){
				  sum -= aft_Buffer[i];
			  }
			  else{
				  sum += aft_Buffer[i];
			  }
		  }
		  first_half_done = 0;
	  }
	  if (second_half_done == 1){
		  for(i = buffer_size/2; i<buffer_size; i++){
			  aft_Buffer[i] = pre_Buffer[i]>>20;
			  if (aft_Buffer[i]>buffmax){
				  buffmax = aft_Buffer[i];
			  }
			  if (aft_Buffer[i]<buffmin){
				  buffmin = aft_Buffer[i];
			  }
			  if(aft_Buffer[i]<0){
				  sum -= aft_Buffer[i];
			  }
			  else{
				  sum += aft_Buffer[i];
			  }
		  }
		  mean = sum/buffer_size;
		  second_half_done = 0;
		  HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  start_recording=0;
	  }
	  if(mean<10){
		  volume_height = 0;
	  }
	  else if(mean<50){
		  volume_height = 1;
	  }
	  else if(mean<100){
		  volume_height = 2;
	  }
	  else if(mean<150){
		  volume_height = 3;
	  }
	  else if(mean<200){
		  volume_height = 4;
	  }
	  else if(mean<250){
		  volume_height = 5;
	  }
	  else if(mean<300){
		  volume_height = 6;
	  }
	  else if(mean<350){
		  volume_height = 7;
	  }
	  else if(mean<400){
		  volume_height = 8;
	  }
	  else if(mean<450){
		  volume_height = 9;
	  }
	  else if(mean<500){
		  volume_height = 10;
	  }
	  else{
		  volume_height = 10;
	  }

  }
  /* USER CODE END StartReadTask */
}

/* USER CODE BEGIN Header_StartKillTask */
/**
* @brief Function implementing the killTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKillTask */
void StartKillTask(void const * argument)
{
  /* USER CODE BEGIN StartKillTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);

//    if (resetCounter==1){
//    	resetCounter = 0;
//    	killCounter =0;
//    }
//    else{
//    	killCounter = killCounter +1;
//    	if (killCounter == 50){ //should be 300
//    		killCounter = 0;
//    		data_index = data_index+1;
//    		if(data_index ==4){
//    			data_index = 0;
//    		}
//    	}
//    }
  }
  /* USER CODE END StartKillTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
