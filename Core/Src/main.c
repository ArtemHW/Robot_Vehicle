/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "mma8452x.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MMA8452X_I2C_ADDRESS (0x1D<<1)
#define WHEEL_q_PERIMETER 51000 //204000  // 204mm or 204000um
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

osThreadId UltrasonicDistanceHandle;
osThreadId LineTrackingHandle;
osThreadId LimitSwitchHandle;
osThreadId InfraredMotionHandle;
osThreadId AccelerometerHandle;
osThreadId MotorsHandle;
osThreadId EncodersHandle;
osThreadId UART_taskHandle;
/* USER CODE BEGIN PV */
typedef struct
{
	QueueHandle_t xQueue1_ultrs;
	uint16_t distance_ultrs;
	TimerHandle_t xTimer1_ultrs;
	char instruction_for_motors[11];
	QueueHandle_t xQueue2_instr4m;
	int x;
	int y;
	int pw;
	uint8_t encod_data[2];
	uint8_t line_data;
	uint32_t encodA_timer[2];
	uint32_t encodB_timer[2];
	int32_t angular_speedA;
	int32_t angular_speedB;
	uint32_t timeCalibA[2];
	uint32_t timeCalibB[2];
	int8_t accelerm_data[6];
	EventGroupHandle_t xEventGroup1;
}buffer_global_type;
buffer_global_type buffer;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM11_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void ultrasonic_dis(void const * argument);
void line_tracking(void const * argument);
void limit_switch(void const * argument);
void infrared_motion(void const * argument);
void accelerometer(void const * argument);
void motors(void const * argument);
void encoders(void const * argument);
void uart_task(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef * htim);
void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef * htim);
void vApplicationIdleHook(void);
void vCallbackFunctionTimer1( TimerHandle_t xTimer );
void UART_RxCallback (UART_HandleTypeDef * huart);
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
	buffer.x = 50;
    buffer.y = 50;
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
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_DMA_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_DeInit(&hi2c1);
  HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);
  GPIOC->ODR |= GPIO_ODR_OD10;
  GPIOC->ODR |= GPIO_ODR_OD12;
  GPIOC->ODR |= GPIO_ODR_OD11;
  GPIOD->ODR |= GPIO_ODR_OD2;
  GPIOB->ODR |= GPIO_ODR_OD15;
  GPIOC->ODR |= GPIO_ODR_OD6;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer.encod_data, 3);

  SCB->CCR |= (1<<1); //Bit 1 USERSETMPEND Enables unprivileged software access to the STIR, see Software trigger interrupt register (NVIC_STIR)

  buffer.xEventGroup1 = xEventGroupCreate();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  buffer.xTimer1_ultrs = xTimerCreate("Timer ultrs trigger", pdMS_TO_TICKS( 40 ), pdTRUE, ( void * ) 0, vCallbackFunctionTimer1);
  xTimerStart(buffer.xTimer1_ultrs, portMAX_DELAY);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  buffer.xQueue1_ultrs = xQueueCreate(10, sizeof(uint16_t));
  buffer.xQueue2_instr4m = xQueueCreate(40, sizeof(uint8_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of UltrasonicDistance */
  osThreadDef(UltrasonicDistance, ultrasonic_dis, osPriorityNormal, 0, 128);
  UltrasonicDistanceHandle = osThreadCreate(osThread(UltrasonicDistance), NULL);

  /* definition and creation of LineTracking */
  osThreadDef(LineTracking, line_tracking, osPriorityNormal, 0, 128);
  LineTrackingHandle = osThreadCreate(osThread(LineTracking), NULL);

  /* definition and creation of LimitSwitch */
  osThreadDef(LimitSwitch, limit_switch, osPriorityNormal, 0, 128);
  LimitSwitchHandle = osThreadCreate(osThread(LimitSwitch), NULL);

  /* definition and creation of InfraredMotion */
  osThreadDef(InfraredMotion, infrared_motion, osPriorityNormal, 0, 128);
  InfraredMotionHandle = osThreadCreate(osThread(InfraredMotion), NULL);

  /* definition and creation of Accelerometer */
  osThreadDef(Accelerometer, accelerometer, osPriorityNormal, 0, 160);
  AccelerometerHandle = osThreadCreate(osThread(Accelerometer), NULL);

  /* definition and creation of Motors */
  osThreadDef(Motors, motors, osPriorityNormal, 0, 400);
  MotorsHandle = osThreadCreate(osThread(Motors), NULL);

  /* definition and creation of Encoders */
  osThreadDef(Encoders, encoders, osPriorityNormal, 0, 200);
  EncodersHandle = osThreadCreate(osThread(Encoders), NULL);

  /* definition and creation of UART_task */
  osThreadDef(UART_task, uart_task, osPriorityNormal, 0, 128);
  UART_taskHandle = osThreadCreate(osThread(UART_task), NULL);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 194;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 31;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 10000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 20;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  USART1->CR1 |= USART_CR1_RXNEIE;
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC15 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef * htim)
{
	if(htim == &htim10 )
	{
		xQueueSendToBackFromISR(buffer.xQueue1_ultrs, (void*)(&(TIM10->CCR1)) ,NULL);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef * htim)
{
	if(htim == &htim11)
	{
		HAL_TIM_PWM_Stop_IT(&htim11, TIM_CHANNEL_1);
	}
}

void vCallbackFunctionTimer1( TimerHandle_t xTimer )
{
	HAL_TIM_PWM_Start_IT(&htim11, TIM_CHANNEL_1);
	xTaskNotifyGive(LineTrackingHandle);
}

void vApplicationIdleHook(void)
{
	__asm__ volatile("NOP");
}

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13) // Freefall/Motion interrupt
	{
		uint8_t result;
		HAL_I2C_Mem_Read(&hi2c1, (0x1D<<1), FF_MT_SRC, 1, &result, sizeof(uint8_t), 100);
		__asm__ volatile("NOP");
	}
	if(GPIO_Pin == GPIO_PIN_14) //Data ready
	{
		mma8452x_ReadData(&hi2c1, MMA8452X_I2C_ADDRESS, buffer.accelerm_data);
	}
}

void UART_RxCallback (UART_HandleTypeDef * huart)
{
	xQueueSendToBackFromISR(buffer.xQueue2_instr4m, (void*)(&(USART1->DR)) ,NULL);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_ultrasonic_dis */
/**
  * @brief  Function implementing the UltrasonicDistance thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ultrasonic_dis */
void ultrasonic_dis(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  xQueueReceive(buffer.xQueue1_ultrs, (void*)(&(buffer.distance_ultrs)), portMAX_DELAY);
	  uint16_t temp1 = buffer.distance_ultrs;
	  xQueueReceive(buffer.xQueue1_ultrs, (void*)(&(buffer.distance_ultrs)), portMAX_DELAY);
	  uint16_t temp2 = buffer.distance_ultrs;
	  uint16_t temp3 = temp2 - temp1;
	  char string_buff[30] = {0};
      sprintf(string_buff, "%d \r\n", temp3);
	  //HAL_UART_Transmit(&huart1, (uint8_t*) string_buff, sizeof(string_buff), 100);
	  xQueueReset(buffer.xQueue1_ultrs);
	  __asm__ volatile("NOP");
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_line_tracking */
/**
* @brief Function implementing the LineTracking thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_line_tracking */
void line_tracking(void const * argument)
{
  /* USER CODE BEGIN line_tracking */
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  int8_t result = 0;
	  result = buffer.line_data;
	  if(result > 100)
	  {
		  xEventGroupClearBits(buffer.xEventGroup1, 0x1);
		TIM3->CCR1 = 1000;
		TIM3->CCR3 = 1000;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		vTaskDelay(250);
		TIM3->CCR1 = 0;
		TIM3->CCR3 = 0;
		 HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		 HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	  }
	  else
	  {
		  xEventGroupSetBits(buffer.xEventGroup1, 0x1);
	  }
	  __asm__ volatile("NOP");
  }
  /* USER CODE END line_tracking */
}

/* USER CODE BEGIN Header_limit_switch */
/**
* @brief Function implementing the LimitSwitch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_limit_switch */
void limit_switch(void const * argument)
{
  /* USER CODE BEGIN limit_switch */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END limit_switch */
}

/* USER CODE BEGIN Header_infrared_motion */
/**
* @brief Function implementing the InfraredMotion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_infrared_motion */
void infrared_motion(void const * argument)
{
  /* USER CODE BEGIN infrared_motion */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END infrared_motion */
}

/* USER CODE BEGIN Header_accelerometer */
/**
* @brief Function implementing the Accelerometer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_accelerometer */
void accelerometer(void const * argument)
{
  /* USER CODE BEGIN accelerometer */
	  HAL_StatusTypeDef result;
	  HAL_StatusTypeDef result2;
	  HAL_I2C_Init(&hi2c1);
	  mma8452x_Standby(&hi2c1, MMA8452X_I2C_ADDRESS);
	  mma8452x_DataFormat(&hi2c1, MMA8452X_I2C_ADDRESS, 1);
	  mma8452x_DataRateSelection(&hi2c1, MMA8452X_I2C_ADDRESS, 4); //Output Data Rate (ODR) 50Hz
	  mma8452x_InterruptPolarityConfig(&hi2c1, MMA8452X_I2C_ADDRESS, 1);
	  mma8452x_InterruptEnable(&hi2c1, MMA8452X_I2C_ADDRESS, EN_FF_MT, CFG_FF_MT);
	  mma8452x_InterruptEnable(&hi2c1, MMA8452X_I2C_ADDRESS, EN_DRDY, CFG_DEFAULT);
	  mma8452x_MotionDetectionConfig(&hi2c1, MMA8452X_I2C_ADDRESS, 0xF8, 0, 20, 2);
	  mma8452x_Active(&hi2c1, MMA8452X_I2C_ADDRESS);
	  taskYIELD();
  /* Infinite loop */
  for(;;)
  {
//	  HAL_I2C_Mem_Read(&hi2c1, (0x1D<<1), CTRL_REG1, 1, buffer.accelerm_data, sizeof(uint8_t), 100);
//	  result = HAL_I2C_Mem_Read(&hi2c1, (0x1D<<1), STATUS, 1, buffer.accelerm_data, sizeof(uint8_t), 100);
//	  HAL_I2C_Mem_Read(&hi2c1, (0x1D<<1), FF_MT_THS, 1, buffer.accelerm_data, sizeof(uint8_t), 100);
//	  HAL_I2C_Mem_Read(&hi2c1, (0x1D<<1), FF_MT_SRC, 1, buffer.accelerm_data, sizeof(uint8_t), 100);
//	  vTaskDelay(5);
//	  mma8452x_ReadData(&hi2c1, MMA8452X_I2C_ADDRESS, buffer.accelerm_data);
//	  vTaskDelay(30);
//	  taskYIELD();
  }
  /* USER CODE END accelerometer */
}

/* USER CODE BEGIN Header_motors */
/**
* @brief Function implementing the Motors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motors */
void motors(void const * argument)
{
  /* USER CODE BEGIN motors */
  /* Infinite loop */
  for(;;)
  {
	  char buff;
	  xQueuePeek(buffer.xQueue2_instr4m, (void*)&buff, portMAX_DELAY);
	  xEventGroupWaitBits(buffer.xEventGroup1, 0x1, pdFALSE, pdTRUE, portMAX_DELAY);
	  if(buff == 'I')
	  {
		  uint8_t i = 0;
		  while(buff != 'i')
		  {
			  xQueuePeek(buffer.xQueue2_instr4m, (void*)&buff, 50);
			  xQueueReceive(buffer.xQueue2_instr4m, (void*)(&(buffer.instruction_for_motors[i])), portMAX_DELAY);
			  i == 10 ? i = 0 : i++;
		  }
		  sscanf(buffer.instruction_for_motors, "IX%dY%di", &buffer.x, &buffer.y);
		  buff = 0;
	  }
	  else
	  {
		  char trash;
		  xQueueReceive(buffer.xQueue2_instr4m, (void*)(&(trash)), 1);
	  }

	  buffer.x = buffer.x - 50;
	  buffer.y = (buffer.y - 50)*-1;
	  buffer.pw = (int)(sqrt(pow(buffer.x,2) + pow(buffer.y,2)));
	  if(buffer.pw > 50) buffer.pw = 50;
	  __asm__ volatile("NOP");

	  if(buffer.y>=0)
	  {
		  if(buffer.x>=0)
		  {
			 TIM3->CCR2 = (250/50)*buffer.pw-(250/50)*buffer.x+750;
			 TIM3->CCR4 = (250/50)*buffer.pw+750;
			 if(buffer.pw <=5)
			 {
				 TIM3->CCR2 = 0;
				 TIM3->CCR4 = 0;
				 HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				 HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
			 }
			 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		  }
		  else if(buffer.x<0)
		  {
			 TIM3->CCR2 = (250/50)*buffer.pw+750;
			 TIM3->CCR4 = (250/50)*buffer.pw+(250/50)*buffer.x+750;
			 if(buffer.pw <=5)
			 {
				 TIM3->CCR2 = 0;
				 TIM3->CCR4 = 0;
				 HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				 HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
			 }
			 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		  }
	  }
	  else if(buffer.y<0)
	  {
		  if(buffer.x>=0)
		  {

		  }
		  else if(buffer.x<0)
		  {

		  }
	  }

//	  switch (buffer.instruction_for_motors) {
//	         case 'U':
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//	             break;
//	         case 'u':
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
//	             break;
//	         case 'D':
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	             break;
//	         case 'd':
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
//	             break;
//	         case 'R':
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	             break;
//	         case 'r':
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
//	             break;
//	         case 'L':
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	             HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//	             break;
//	         case 'l':
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//	             HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
//	             break;
//	         default:
//	        	 __asm__ volatile("NOP");
//	     }
  }
  /* USER CODE END motors */
}

/* USER CODE BEGIN Header_encoders */
/**
* @brief Function implementing the Encoders thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoders */
void encoders(void const * argument)
{
  /* USER CODE BEGIN encoders */
	uint8_t i, ii = 0;
  /* Infinite loop */
  for(;;)
  {
	  //Diameter of wheel is 65mm          80 180
	  if(buffer.encod_data[0]>=180)
	  {
		  buffer.encodA_timer[0] = HAL_GetTick();
	  }
	  else if(buffer.encod_data[0]<=80)
	  {
		  buffer.encodA_timer[1] = HAL_GetTick();
	  }
	  if((buffer.encodA_timer[0] && buffer.encodA_timer[1]) != 0)
	  {
		  buffer.angular_speedA = WHEEL_q_PERIMETER/(2*(int32_t)(buffer.encodA_timer[0] - buffer.encodA_timer[1]));
		  memset(buffer.encodA_timer, 0, sizeof(buffer.encodA_timer));
	  }
	  if(buffer.encod_data[1]>=180)
	  {
		  buffer.encodB_timer[0] = HAL_GetTick();
	  }
	  else if(buffer.encod_data[1]<=80)
	  {
		  buffer.encodB_timer[1] = HAL_GetTick();
	  }
	  if((buffer.encodB_timer[0] && buffer.encodB_timer[1]) != 0)
	  {
		  buffer.angular_speedB = WHEEL_q_PERIMETER/(2*(int32_t)(buffer.encodB_timer[0] - buffer.encodB_timer[1]));
		  memset(buffer.encodB_timer, 0, sizeof(buffer.encodB_timer));
	  }
	  vTaskDelay(4);
  }
  /* USER CODE END encoders */
}

/* USER CODE BEGIN Header_uart_task */
/**
* @brief Function implementing the UART_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_task */
void uart_task(void const * argument)
{
  /* USER CODE BEGIN uart_task */
  /* Infinite loop */
  for(;;)
  {
//	  char string_buff[20] = {0};
//	  sprintf(string_buff, " %d %d ", buffer.encod_data[0], buffer.encod_data[1]);
//	  HAL_UART_Transmit(&huart1, (uint8_t*) string_buff, sizeof(string_buff), 100);
	  char string_buff2[20] = {0};
	  sprintf(string_buff2, " %d %d %d ", buffer.accelerm_data[0], buffer.accelerm_data[1], buffer.accelerm_data[2]);
	  HAL_UART_Transmit(&huart1, (uint8_t*) string_buff2, sizeof(string_buff2), 100);
	  char string_buff3[20] = {0};
	  sprintf(string_buff3, " %d %d \n", buffer.angular_speedA, buffer.angular_speedB);
	  HAL_UART_Transmit(&huart1, (uint8_t*) string_buff3, sizeof(string_buff3), 100);
	  vTaskDelay(160);
  }
  /* USER CODE END uart_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
