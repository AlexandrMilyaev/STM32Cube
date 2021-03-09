/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_flash_ex.h"
#include "stm32f1xx_hal_flash.h"
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
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
tdStructUL212 ul212;
FLASH_EraseInitTypeDef hflash;
uint32_t hPageError;
uint32_t periodLed;
uint32_t counterLed;
uint32_t counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void UL212_Init(tdStructUL212 * UL212);
void UL212_LedInit(tdEnumStatusLed  STATUS_LED);
uint8_t AsciiToHex (char simbol);
void UL212_DataParser(tdStructUL212 * UL212);
void UL212_LedHandler(tdStructUL212 * UL212);
void UL212_FrequencyOut(int32_t frequency);
void FlashWriteSettings(uint32_t data, uint32_t page);
uint32_t FlashRead (uint32_t addr);
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  UL212_Init(&ul212);
  HAL_GPIO_WritePin(BUTTON_OUT_GPIO_Port, BUTTON_OUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart1, ul212.rawData.uartBuffer, 1);
  //HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
  //HAL_TIMEx_OCN_Start_IT(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (ul212.rawData.uartFlagEndString)
		  UL212_DataParser(&ul212);
	  if (ul212.hardwareCode == HW_NORMAL && ul212.softwareCode == SW_NORMAL_DATA &&
			  ul212.signalStrength > 60 && ul212.tiltAngle < 10 && ul212.validSignal > 26){
		  uint16_t dataL = ul212.liquidLevelValue;
		  UL212_FrequencyOut((1000*dataL/ul212.valueUp) + 500);
		  //UL212_FrequencyOut(((float)dataL*ul212.coeficient) + 500);
	  }

	  UL212_LedHandler(&ul212);
	  if (counter){
		  if(HAL_GPIO_ReadPin(BUTTON_IN_GPIO_Port, BUTTON_IN_Pin) != GPIO_PIN_SET){
			  if(ul212.hardwareCode == HW_NORMAL && ul212.softwareCode == SW_NORMAL_DATA){
				  ul212.valueUp = ul212.liquidLevelValue;
				  ul212.coeficient = (float)1000/(float)ul212.valueUp;
				  FlashWriteSettings(ul212.valueUp, SETTINGS_PAGE);
				  counter = 0;
				  UL212_LedInit(LED_FLASH_PROGRAM_OK);
			  }
		  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 48;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 24;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUTTON_OUT_GPIO_Port, BUTTON_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_OUT_Pin */
  GPIO_InitStruct.Pin = BUTTON_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUTTON_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_IN_Pin */
  GPIO_InitStruct.Pin = BUTTON_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	counter++;
}

void ButtonHandling(tdStructUL212 * UL212){}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch (*ul212.rawData.uartBuffer)
	{
		case '*':
		{
			if (ul212.statusLed == LED_OK)HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			ul212.rawData.uartPoint = 0;
			ul212.rawData.uartBufferString[ul212.rawData.uartPoint] = *ul212.rawData.uartBuffer;
			ul212.rawData.uartPoint++;
		}
			break;

		case '#':
		{
			if(ul212.statusLed == LED_OK)HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			ul212.rawData.uartFlagEndString = TRUE;
			ul212.rawData.uartBufferString[ul212.rawData.uartPoint] = *ul212.rawData.uartBuffer;
			ul212.rawData.uartPoint++;
		}
			break;
		default:
		{
			ul212.rawData.uartBufferString[ul212.rawData.uartPoint] = *ul212.rawData.uartBuffer;
			ul212.rawData.uartPoint++;
		}
			break;
	}

	HAL_UART_Receive_IT(&huart1, ul212.rawData.uartBuffer, 1);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

void UL212_DataParser(tdStructUL212 * UL212)
{
	tdEnumDataParser dataParser = START;
	uint32_t temp;
	uint8_t parserPoint;
	for (parserPoint = 0; UL212->rawData.uartBufferString[parserPoint] != '#'; parserPoint++) {
		if (UL212->rawData.uartBufferString[parserPoint] == ',')
		{
			dataParser++;
			switch (dataParser) {
				case VERSION:
					UL212->protocolVersion = UL212->rawData.uartBufferString[parserPoint+1] - 0x30;
					UL212->firmwareVersion = ((UL212->rawData.uartBufferString[parserPoint+2] - 0x30) * 10) +
							+ (UL212->rawData.uartBufferString[parserPoint+3]-0x30);
					UL212->hardwareVersion = UL212->rawData.uartBufferString[parserPoint+4] - 0x30;
				break;
				case MINUTE:
					UL212->minute = ((UL212->rawData.uartBufferString[parserPoint+1] - 0x30) * 10) +
							+ (UL212->rawData.uartBufferString[parserPoint+2] - 0x30);
					break;
				case LIQUID_LEVEL_VALUE:
					UL212->liquidLevelValue = ((UL212->rawData.uartBufferString[parserPoint+1] - 0x30) * 1000) +
							+ ((UL212->rawData.uartBufferString[parserPoint+2] - 0x30) * 100) +
							+ ((UL212->rawData.uartBufferString[parserPoint+3] - 0x30) * 10) +
							+ (UL212->rawData.uartBufferString[parserPoint+4] - 0x30);
					break;
				case SIGNAL_STRENGTH:
					UL212->signalStrength = ((UL212->rawData.uartBufferString[parserPoint+1] - 0x30) * 10) +
							+ (UL212->rawData.uartBufferString[parserPoint+2] - 0x30);
					UL212->softwareCode = UL212->rawData.uartBufferString[parserPoint+3] - 0x30;
					UL212->hardwareCode = UL212->rawData.uartBufferString[parserPoint+4] - 0x30;
					break;
				case REAL_TIME_VALUE:
					UL212->realTimeValue = ((UL212->rawData.uartBufferString[parserPoint+1] - 0x30) * 1000) +
							+ ((UL212->rawData.uartBufferString[parserPoint+2] - 0x30) * 100) +
							+ ((UL212->rawData.uartBufferString[parserPoint+3] - 0x30) * 10) +
							+ (UL212->rawData.uartBufferString[parserPoint+4] - 0x30);
					break;
				case TEMPERATURE:
					temp = ((UL212->rawData.uartBufferString[parserPoint+1] - 0x30) * 1000) +
							+ ((UL212->rawData.uartBufferString[parserPoint+2] - 0x30) * 100) +
							+ ((UL212->rawData.uartBufferString[parserPoint+3] - 0x30) * 10) +
							+ (UL212->rawData.uartBufferString[parserPoint+4] - 0x30);
					UL212->temperature = (temp - 400) * 0.1;
					break;
				case VALID_SIGNAL:
					UL212->validSignal = ((UL212->rawData.uartBufferString[parserPoint+1] - 0x30) * 10) +
							+ (UL212->rawData.uartBufferString[parserPoint+2] - 0x30);
					UL212->tiltAngle = (AsciiToHex(UL212->rawData.uartBufferString[parserPoint+3]) * 0x0f) +
							AsciiToHex(UL212->rawData.uartBufferString[parserPoint+4]);
				    break;
				default:
					break;
			}
		}
	}
	UL212->rawData.uartFlagEndString = FALSE;
}


void UL212_LedInit(tdEnumStatusLed  STATUS_LED)
{
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  ul212.statusLed = STATUS_LED;
	  ul212.counterLed = STATUS_LED * 2;
}

void UL212_LedHandler(tdStructUL212 * UL212)
{


	switch (UL212->statusLed) {
		case LED_FLASH_PROGRAM_OK:
			if (HAL_GetTick() > periodLed)
			{
				periodLed = HAL_GetTick() + LED_DELAY;
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				UL212->counterLed--;

			}
			break;
		default:
			break;
	}
	if (UL212->counterLed == 0)UL212->statusLed = LED_OK;
}

void UL212_FrequencyOut(int32_t frequency)
{
	//int32_t prescellar = -1 * frequency + 2000;
	int32_t prescellar = 72000000/(frequency * 2 * 48);// 48 : Предделитель таймера
	TIM4->PSC = prescellar - 1;
}

void UL212_Init(tdStructUL212 * UL212)
{
	UL212->protocolVersion = 0;
	UL212->firmwareVersion = 0;
	UL212->hardwareVersion = 0;
	UL212->minute = 0;
	UL212->liquidLevelValue = 1600;
	UL212->signalStrength = 0;
	UL212->softwareCode = SW_WEAK_SIGNAL;
	UL212->hardwareCode = HW_SENSOR_DISCONNECT;
	UL212->realTimeValue = 0;
	UL212->temperature = 0.0;
	UL212->validSignal = 0;
	UL212->tiltAngle = 0;
	UL212->calibration = NOT_READY;
	UL212->valueUp = FlashRead(SETTINGS_PAGE);
	UL212->coeficient = (float)1000/(float)UL212->valueUp;

}

uint8_t AsciiToHex (char simbol)
{
	switch (simbol) {
		case '0': return 0x00;
		case '1': return 0x01;
		case '2': return 0x02;
		case '3': return 0x03;
		case '4': return 0x04;
		case '5': return 0x05;
		case '6': return 0x06;
		case '7': return 0x07;
		case '8': return 0x08;
		case '9': return 0x09;
		case 'a': case 'A': return 0x0a;
		case 'b': case 'B': return 0x0b;
		case 'c': case 'C': return 0x0c;
		case 'd': case 'D': return 0x0d;
		case 'e': case 'E': return 0x0e;
		case 'f': case 'F': return 0x0f;
		default: return 0xff;
	}
}

void FlashWriteSettings(uint32_t data, uint32_t page)
{
	HAL_FLASH_Unlock();
	hflash.TypeErase = FLASH_TYPEERASE_PAGES;
	hflash.PageAddress = page;
	hflash.NbPages = 1;
	FLASH_WaitForLastOperation(500);
	HAL_FLASHEx_Erase(&hflash, &hPageError);
	FLASH_WaitForLastOperation(500);
	while(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, page, data) != HAL_OK);
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Lock();


}

uint32_t FlashRead (uint32_t addr)
{
	return (*(__IO uint32_t*) addr);
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
