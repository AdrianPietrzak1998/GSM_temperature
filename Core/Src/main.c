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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ring_buffer.h"
#include "string.h"
#include "parser_complex.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INQUIRY_TIME 750
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RingBuffer_t ReceiveBuffer;
uint8_t ReceiveTmp;
uint8_t LineCounter = 0;
uint8_t ReceivedData[255];

uint8_t Uart1isBusy = 0;
uint8_t *Uart1isBusyPtr = &Uart1isBusy;

uint32_t LastTickForSim800;

double SignalQuality;
uint8_t ReceivedState;
uint8_t CRegN, CRegStat;

//time variables
uint8_t year, month, day, hour, minute, second;

const char ctrlZ = 26;

SMSUartTxState_t SMSUartTxState;

uint8_t timPeriodCounter = 0;
uint16_t inquiryTimeVar = INQUIRY_TIME;

char SMSMessage[140] = "TEest.asdqwejkshda";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &ReceiveTmp, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
  SMSUartTxState = Config;
  LastTickForSim800 = HAL_GetTick();
  while (1)
  {

	  if(LineCounter)
	  {
		  Parser_TakeLine(&ReceiveBuffer, ReceivedData);

		  LineCounter--;

		  Parser_parse(ReceivedData);
	  }

	  if(Uart1isBusy)
	  {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	  }


	  if(Uart1isBusy == 0 && HAL_GetTick() - LastTickForSim800 >= inquiryTimeVar)
	  	{LastTickForSim800 = HAL_GetTick();
	  		if(SMSUartTxState == Control)
	  		{
	  			static uint8_t TaskState = 0;

	  			switch(TaskState)
	  			{
	  			case 0:
	  				UartSend("AT+CSQ\r\n");
	  				TaskState = 1;
	  				break;
	  			case 1:
	  				UartSend("AT+CREG?\r\n");
	  				TaskState = 2;
	  				break;
	  			case 2:
	  				UartSend("AT+CCLK?\r\n");
	  				TaskState = 0;
	  				SMSUartTxState = Control;
	  				break;
	  			}
	  		}
	  		else if(SMSUartTxState == SMSMsgWrite)
	  		{
	  			static uint8_t TaskState = 0;
	  			switch(TaskState)
	  			{
	  			case 0:
	  				UartSendWoRxCtrl("AT+CMGS=\"+48885447216\"\r\n");
	  				inquiryTimeVar = 2000;
	  				TaskState = 1;
	  				break;
	  			case 1:
	  				HAL_UART_Transmit_IT(&huart1, (uint8_t*) SMSMessage, strlen(SMSMessage));
	  				*Uart1isBusyPtr = 1;
	  				TaskState = 2;
	  				break;
	  			case 2:
	  				HAL_UART_Transmit_IT(&huart1, (uint8_t*) &ctrlZ, 1);
	  				*Uart1isBusyPtr = 1;
	  				TaskState = 3;
	  				SMSUartTxState = Control;
	  				break;
	  			case 3:
	  				inquiryTimeVar = INQUIRY_TIME;
	  				TaskState = 0;
	  				break;
	  			}
	  		}
	  		else if(SMSUartTxState == Config)
	  		{
	  			static uint8_t TaskState = 0;
	  			switch(TaskState)
	  			{
	  			case 0:
	  				UartSend("ATE0\r\n");
	  				TaskState = 1;
	  				break;
	  			case 1:
	  				UartSend("AT+CMGF=1\r\n");
	  				TaskState = 2;
	  				break;
	  			case 2:
	  				UartSend("AT+CLTS=1\r\n");
	  				TaskState = 3;
	  				break;
	  			case 3:
	  				UartSend("AT+CNMI=2,2,0,0,0\r\n");
	  				TaskState = 4;
	  				break;
	  			case 4:
	  				UartSend("AT&W\r\n");
	  				TaskState = 0;
	  				SMSUartTxState = Control;
	  				break;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		if(ReceiveTmp != 0xd)
		{
			if (RB_OK == Ring_Buffer_Write(&ReceiveBuffer, ReceiveTmp))
			{
				if(ReceiveTmp == ENDLINE)
				{
					LineCounter++;
				}
			}
		}
		HAL_UART_Receive_IT(&huart1, &ReceiveTmp, 1);
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	*Uart1isBusyPtr = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	if(htim->Instance == TIM4 && Uart1isBusy == 0)
//	{
//		if(SMSUartTxState == Control)
//		{
//			static uint8_t TaskState = 0;
//
//			switch(TaskState)
//			{
//			case 0:
//				UartSend("AT+CSQ\r\n");
//				TaskState = 1;
//				break;
//			case 1:
//				UartSend("AT+CREG?\r\n");
//				TaskState = 2;
//				break;
//			case 2:
//				UartSend("AT+CCLK?\r\n");
//				TaskState = 0;
//			}
//		}
//		else if(SMSUartTxState == SMSMsgWrite)
//		{
//			static uint8_t TaskState = 0;
//			switch(TaskState)
//			{
//			case 0:
//				UartSendWoRxCtrl("AT+CMGS=\"+48885447216\"\r\n");
//				TaskState = 1;
//				break;
//			case 1:
//				HAL_UART_Transmit_IT(&huart1, (uint8_t*) SMSMessage, strlen(SMSMessage));
//				*Uart1isBusyPtr = 1;
//				TaskState = 2;
//				break;
//			case 2:
//				HAL_UART_Transmit_IT(&huart1, (uint8_t*) &ctrlZ, 1);
//				TaskState = 0;
//				SMSUartTxState = Control;
//				break;
//				}
//		}
//		else if(SMSUartTxState == Config)
//		{
//			static uint8_t TaskState = 0;
//			switch(TaskState)
//			{
//			case 0:
//				UartSend("ATE0\r\n");
//				TaskState = 1;
//				break;
//			case 1:
//				UartSend("AT+CMGF=1\r\n");
//				TaskState = 2;
//				break;
//			case 2:
//				UartSend("AT+CLTS=1\r\n");
//				TaskState = 3;
//				break;
//			case 3:
//				UartSend("AT+CNMI=2,2,0,0,0\r\n");
//				TaskState = 4;
//				break;
//			case 4:
//				UartSend("AT&W\r\n");
//				TaskState = 0;
//				SMSUartTxState = Control;
//				break;
//
//			}
//		}
//	}
	if(htim->Instance == TIM3)
	{
		//Period elapsed 128,57s
		timPeriodCounter++;



		if(timPeriodCounter == 28)
		{
			timPeriodCounter = 0;
		}
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
