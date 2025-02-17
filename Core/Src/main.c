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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#include "usbd_hid.h"
#include "math.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define DEBUG_PRINT(...) do{\
	uint32_t tick = HAL_GetTick();\
	printf("[%6lu.%03lu]",tick/1000, tick%1000);\
	printf(__VA_ARGS__);\
	}while(0)


typedef struct {
	float x;
	float y;
	float z;
}Coordinate_t;

int8_t buffer[4] = {0,};
enum {
	click_e = 0,
	x_e,
	y_e,
	wheel_e
};
const float sigma = 10.0f;
const float beta = 8/3.0f;
const float rho = 28.0f;

Coordinate_t p = {1.0f, 1.0f, 1.0f};
Coordinate_t d = {0.0f, 0.0f, 0.0f};
Coordinate_t ip = {0.0f, 0.0f, 0.0f};
Coordinate_t id = {0.0f, 0.0f, 0.0f};

const float dt = 0.005f;

Coordinate_t plotCurr = {0.0f, 0.0f, 0.0f};
Coordinate_t plotPrev = {0.0f, 0.0f, 0.0f};
Coordinate_t plotDelta = {0.0f, 0.0f, 0.0f};

static uint32_t delay 	= 30;
static float 	amp 	= 2.0f;
int working_flag = 1;
int detect_flag = 0;
int expired_flag = 0;
const uint32_t expired_tick = 1000*60*60*9; // Approx. 9 Hours

Coordinate_t isometric_projection(Coordinate_t point){
	Coordinate_t ret = {0.0f, 0.0f, 0.0f};
	const static float rot_matrix[3][3] = {
		{sqrt(3)/sqrt(6), 		0, 					-1*sqrt(3)/sqrt(6)},
		{1/sqrt(6), 			2/sqrt(6), 	 		1/sqrt(6)},
		{sqrt(2)/sqrt(6), 		-1*sqrt(2)/sqrt(6), sqrt(2)/sqrt(6)}
	};
	ret.x = rot_matrix[0][0]*point.x + rot_matrix[0][1]*point.y + rot_matrix[0][2]*point.z;
	ret.y = rot_matrix[1][0]*point.x + rot_matrix[1][1]*point.y + rot_matrix[1][2]*point.z;
	ret.z = rot_matrix[2][0]*point.x + rot_matrix[2][1]*point.y + rot_matrix[2][2]*point.z;

	return ret;
}

void iterate(){
	/* Calculate Lorentz attractor */
	d.x = sigma*(p.y - p.x);
	d.y = p.x*(rho - p.z) - p.y;
	d.z = p.x*p.y - beta*p.z;
	d.x *= dt;
	d.y *= dt;
	d.z *= dt;
	p.x += d.x;
	p.y += d.y;
	p.z += d.z;
}


uint32_t RCC_PrintFlag(){
	uint32_t ret = 0;
	DEBUG_PRINT("Print ResetFlag\n");
	if ( __HAL_RCC_GET_FLAG(PWR_FLAG_SB)){
		DEBUG_PRINT("System resumed from STANDBY mode\n");
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == SET)
	{
		DEBUG_PRINT("Software Reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) == SET)
	{
		DEBUG_PRINT("Power on reset\n"); ret++;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == SET)
	{
		DEBUG_PRINT("External reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == SET)
	{
		DEBUG_PRINT("WDT Reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) == SET)
	{
		DEBUG_PRINT("Window WDT Reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == SET)
	{
		DEBUG_PRINT("Low Power Reset\n"); ret++;
	}
	//		if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) // F4 Usually set with POR
	//		{
	//			printf("Brown-Out Reset\n"); ret++;
	//		}

	/* Clear source Reset Flag */
	__HAL_RCC_CLEAR_RESET_FLAGS();
	return ret;
}

uint32_t read_bkup_reg(){
	static int addr = 11;
	// Read
	uint32_t value = HAL_RTCEx_BKUPRead(&hrtc, addr);
	// Store toggled value
	HAL_RTCEx_BKUPWrite(&hrtc, addr, !value);
	// return
	return !value;
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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_EXTI_Callback(RADAR_OUT_Pin); // Update first status

  uint8_t buffer[4];
  buffer[0]=0;//buttons first 3 bits
  buffer[1]=100;//X axis 8bit value signed
  buffer[2]=0;//Y axis 8bit value signed
  buffer[3]=0;//Wheel 8bit value signed


	DEBUG_PRINT("Lorenz Attractor\n");
	HAL_Delay(3000);

	DEBUG_PRINT("Waiting for 2seconds... \n");
	for(int i = 0; i < 20; i++){
		HAL_GPIO_TogglePin(LD0_GPIO_Port, LD0_Pin);
		HAL_Delay(100);
	}

	DEBUG_PRINT("Working Test...\n");
	for(int i = 0; i< 20; i++){
		switch(i%4){
		case 0:
			buffer[x_e] = 100;
			buffer[y_e] = 0;
			break;
		case 1:
			buffer[x_e] = 0;
			buffer[y_e] = 100;
			break;
		case 2:
			buffer[x_e] = -100;
			buffer[y_e] = 0;
			break;
		case 3:
			buffer[x_e] = 0;
			buffer[y_e] = -100;
			break;
		}
		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)buffer, 4);
		HAL_Delay(100);
	}
	DEBUG_PRINT("Running Lorenz Attractor...\n");

	while(1)
	{
//		for(int i = 0; i<100000; i++)
		iterate();

		/* Print Axis Value */
#if 0
		DEBUG_PRINT(" x:%6.03f\t y:%6.03f\t z:%6.03f\t\n",p.x,p.y,p.z);
		DEBUG_PRINT("dx:%6.03f\tdy:%6.03f\tdz:%6.03f\t\n",d.x,d.y,d.z);
#endif

		/* Isometric Rotation */
		ip = isometric_projection(p);
#if 0
		DEBUG_PRINT(" x:%6.03f\t y:%6.03f\t z:%6.03f\t\n",ip.x,ip.y,ip.z);
#endif

		/* Plotting */
		plotCurr = ip;
		plotCurr.x *= amp;
		plotCurr.y *= amp;
		plotCurr.z *= amp;
		// To digitize int
		plotDelta.x = roundf(plotCurr.x - plotPrev.x);
		plotDelta.y = roundf(plotCurr.y - plotPrev.y);
		plotDelta.z = roundf(plotCurr.z - plotPrev.z);

		plotPrev.x += plotDelta.x;
		plotPrev.y += plotDelta.y;
		plotPrev.z += plotDelta.z;
#if 0
		DEBUG_PRINT(" x:%6.03f\t y:%6.03f\t z:%6.03f\t\n",plotCurr.x,plotCurr.y,plotCurr.z);
		DEBUG_PRINT("dx:%6.03f\tdy:%6.03f\tdz:%6.03f\t\n",plotDelta.x,plotDelta.y,plotDelta.z);
		printf("\n");
#endif

		/* Send USB Buffer */
		buffer[x_e] = (int8_t) (plotDelta.x);
		buffer[y_e] = (int8_t) (plotDelta.y);

		uint32_t curr_tick = HAL_GetTick();
		if(curr_tick > expired_tick)
		{
			expired_flag = 1;
			working_flag = 0;
		}

		if(working_flag | (!detect_flag))
		{
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)buffer, 4);
		} else {
			// Just Do Nothing.

			const uint8_t buffer_empty[4] = {0,0,0,0};
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)buffer_empty, 4);

		}


		/* Delay */
		HAL_Delay(delay);

		/* Working LED Indicator */
		static uint32_t i = 0;
		if(!(i++%(100/delay))){ // about 10Hz
			static uint8_t blink[20] = {0,1,0,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1};
			static uint8_t blink_nw[20] = {0,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1};
			static uint8_t blink_detect[20] = {0,1,0,1,0, 0,0,1,1,1, 1,1,1,1,1, 1,1,1,1,1};
			static uint8_t blink_expired[20] = {0,0,0,0,0, 0,0,0,0,0, 1,1,1,1,1, 1,1,1,1,1};
			static uint8_t cnt = 0;

			if(!detect_flag)
			{
				HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, blink_detect[cnt]);
			} else if(working_flag){
				HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, blink[cnt]);
			} else if(expired_flag){
				HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, blink_expired[cnt]);
			} else {
				HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, blink_nw[cnt]);
			}
			cnt++;
			if(sizeof(blink) <= cnt)
				cnt = 0;
//			DEBUG_PRINT("Led Toggle\n");
		}

	}

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_VCC_Pin|ADC_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD0_Pin */
  GPIO_InitStruct.Pin = LD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B0_Pin */
  GPIO_InitStruct.Pin = B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_VCC_Pin ADC_GND_Pin */
  GPIO_InitStruct.Pin = ADC_VCC_Pin|ADC_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RADAR_OUT_Pin */
  GPIO_InitStruct.Pin = RADAR_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RADAR_OUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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
