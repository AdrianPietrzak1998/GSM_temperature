/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
typedef enum
{
	Reset=0,
	Start,
	Config,
	Control,
	SMSMsgWrite,
	FTPMsgWrite,
	Idle
}SMSUartTxState_t;


typedef struct {
	double SignalQuality;
	uint8_t ReceivedState;
	uint8_t CRegN, CRegStat;
	struct FtpPut
	{
		uint8_t mode;
		uint8_t status;
		uint16_t maxLength;
		uint16_t CnfLength;
	}FtpPut;

	char SMSNumber[16];

	union
	{
		uint32_t FlashBuff[128];
		struct ConfigFlash
		{
			char login[30];
			char password[30];
			char server[50];
			char path[50];
			char number1[16];
			char number2[16];
			char deviceNumber[5];
			char apn[32];
		}ConfigFlash;
	};
	uint8_t ErrorCounter;
	uint32_t LastTickReceive;
	struct TaskToDo
	{
		uint8_t FtpMsgToSend:1;
		uint8_t SmsMsgToSend:1;
	} TaskToDo;

	uint16_t ResetCounter;

}GSM_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_14
#define BUTTON_GPIO_Port GPIOC
#define GSM_RESET_Pin GPIO_PIN_12
#define GSM_RESET_GPIO_Port GPIOB
#define DS18B20_Pin GPIO_PIN_7
#define DS18B20_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SMS_SIZE 240
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
