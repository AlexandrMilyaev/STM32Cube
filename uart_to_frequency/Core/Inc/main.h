/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BUTTON_OUT_Pin GPIO_PIN_12
#define BUTTON_OUT_GPIO_Port GPIOB
#define BUTTON_IN_Pin GPIO_PIN_13
#define BUTTON_IN_GPIO_Port GPIOB
#define BUTTON_IN_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */
#define SETTINGS_PAGE 0x08007c00
#define LED_DELAY  500


typedef enum{
	LED_OK,
	LED_FLASH_PROGRAM_OK
}tdEnumStatusLed;


typedef enum {
	NOT_READY,
	NOT_CALIBRATED,
	OK_CALIBRATED
}tdEnumCal;

typedef enum {
	FALSE,
	TRUE
}tdEnumFlag;


typedef enum {
	START,
	VERSION,
	MINUTE,
	LIQUID_LEVEL_VALUE,
	SIGNAL_STRENGTH,
	REAL_TIME_VALUE,
	TEMPERATURE,
	VALID_SIGNAL
}tdEnumDataParser;

typedef enum{
	SW_NORMAL_DATA,
	SW_POWER_ON,
	SW_WEAK_SIGNAL
}tdSWCode;

typedef enum{
	HW_NORMAL,
	HW_SENSOR_DISCONNECT,
	HW_SENSOR_FALL_OFF,
	HW_SYSTEM_VOLTAGE_ANOMALY

}tdHWCode;

typedef struct{
	uint8_t uartBuffer[1];
	uint8_t uartBufferString[50];
	uint8_t uartPoint;
	tdEnumFlag uartFlagEndString;
}tdRawData;

typedef struct{
	tdRawData rawData;
	uint8_t protocolVersion;
	uint8_t firmwareVersion;
	uint8_t hardwareVersion;
	uint8_t minute;
	uint16_t liquidLevelValue;
	uint8_t signalStrength;
	tdSWCode softwareCode;
	tdHWCode hardwareCode;
	uint16_t realTimeValue;
	float temperature;
	uint8_t validSignal;
	uint16_t tiltAngle;
	uint32_t topValue;
	tdEnumStatusLed statusLed;
	uint32_t counterLed;
	tdEnumCal calibration;
	uint16_t valueUp;
	float coeficient;


}tdStructUL212;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
