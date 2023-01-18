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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ring_buffer.h"
#include "parser_complex.h"
#include "utils.h"
#include "stdio.h"
#include "math.h"
#include "ds18b20.h"
#include "string.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INQUIRY_TIME 140
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

RingBuffer_t USBConfBuffer;
uint8_t USBReceiveTmp;
uint8_t USBLineCounter = 0;
//uint8_t USBReceivedData [255];

uint8_t Uart1isBusy = 0;
uint8_t *Uart1isBusyPtr = &Uart1isBusy;

uint32_t LastTickForSim800;
uint32_t LastTickTempMeasure;




//time variables
uint8_t year, month, day, hour, minute, second;

int32_t temperature;


const char ctrlZ = 26;

SMSUartTxState_t SMSUartTxState;
GSM_t GSM;


uint8_t timPeriodCounter = 0;
uint16_t inquiryTimeVar = INQUIRY_TIME;

char SMSMessage[SMS_SIZE];
char FTPMessageBox1[1330];
char FTPMessageBox2[1330];
uint8_t FTPMessageBoxRecordSwitch = 1;

uint8_t ds_address[DS18B20_ROM_CODE_SIZE];

//volatile uint8_t USBtoParseBuf[512];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void CommStateMachineTask(void);
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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &ReceiveTmp, 1);
  if (ds18b20_read_address(ds_address) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SMSUartTxState = Reset;
  LastTickForSim800 = HAL_GetTick();
  LastTickTempMeasure = HAL_GetTick();

  memcpy(GSM.FlashBuff, 0x0801FC00, 128*4);

//  sprintf(GSM.ConfigFlash.server, "www.mkwk019.cba.pl");
//  sprintf(GSM.ConfigFlash.login, "hostv1@donakoemb.cba.pl");
//  sprintf(GSM.ConfigFlash.password, "FalconEye2022");
//  sprintf(GSM.ConfigFlash.path, "/czujnik/");

  ///Test zapisu do flash

//  Flash_Write_Data(0x0801FC00, GSM.FlashBuff, 128);

  HAL_TIM_Base_Start_IT(&htim3);

  while (1)
  {
//	  HAL_IWDG_Refresh(&hiwdg);

	  if(HAL_GetTick() - LastTickTempMeasure >= 800)
	  {
		  static uint8_t tempMeasureFlag = 0;
		  if(!tempMeasureFlag)
		  {
			  ds18b20_start_measure(NULL);
			  tempMeasureFlag = 1;
		  }
		  else
		  {
			  temperature = ds18b20_get_temp_wo_fp(NULL);
			  tempMeasureFlag = 0;
		  }

	  }

	  if(LineCounter)
	  {
		  Parser_TakeLine(&ReceiveBuffer, ReceivedData);

		  LineCounter--;

		  Parser_parse(ReceivedData);
	  }

	  if(USBLineCounter)
	  {
		  Parser_TakeLine(&USBConfBuffer, ReceivedData);

		  USBLineCounter--;

		  Parser_parse(ReceivedData);
	  }


	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GSM.ReceivedState);


	  CommStateMachineTask();

	  if(GSM.ErrorCounter > 10)
	  {
		  SMSUartTxState = Reset;
		  GSM.ErrorCounter = 0;
	  }

	  if(SMSUartTxState != Idle && HAL_GetTick() - GSM.LastTickReceive >= 30000)
	  {
		  SMSUartTxState = Reset;
		  GSM.LastTickReceive = HAL_GetTick();
	  }

	  if(!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))
	  {
		  sprintf(SMSMessage, "%.1f\n%d\n%s\n%s\n%s", GSM.SignalQuality, GSM.ErrorCounter, GSM.ConfigFlash.apn, GSM.ConfigFlash.path, GSM.ConfigFlash.server);
		  strcpy(GSM.SMSNumber, GSM.ConfigFlash.number2);
		  GSM.TaskToDo.SmsMsgToSend = 1;
	  }

	  if(GSM.CRegStat == 2)
		  SMSUartTxState = Start;




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		GSM.LastTickReceive = HAL_GetTick();
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
	if(htim->Instance == TIM3)
	{   //Period elapsed 128,57s
		timPeriodCounter++;
		char OneSample[32];
		char TemperatureString[7];
		Temperature100ToString(temperature, TemperatureString);
		sprintf(OneSample, "%.2u/%.2u/%.2u,%.2u:%.2u:%.2u,%s\n", year, month, day, hour, minute, second, TemperatureString);
		if(FTPMessageBoxRecordSwitch == 1)
		{
			strcat(FTPMessageBox1, OneSample);
		}
		else if(FTPMessageBoxRecordSwitch == 2)
		{
			strcat(FTPMessageBox2, OneSample);
		}


		if(timPeriodCounter == 42)
		{
//			SMSUartTxState = FTPMsgWrite;
			GSM.TaskToDo.FtpMsgToSend = 1;

			if(FTPMessageBoxRecordSwitch == 1)
			{
				FTPMessageBox2[0] = '\0';
				FTPMessageBoxRecordSwitch = 2;
			}
			else if(FTPMessageBoxRecordSwitch == 2)
			{
				FTPMessageBox1[0] = '\0';
				FTPMessageBoxRecordSwitch = 1;
			}

			timPeriodCounter = 0;
		}
	}
}

void CommStateMachineTask(void)
{
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
	  				 UartSend("AT+CMGL=\"REC UNREAD\",0\r\n");
	  				 TaskState = 3;
	  				 break;
	  			case 3:
	  				UartSend("AT+CCLK?\r\n");
	  				TaskState = 0;
	  				if(!GSM.TaskToDo.FtpMsgToSend && !GSM.TaskToDo.SmsMsgToSend)
	  				{
	  					SMSUartTxState = Control;
	  				}
	  				else if(GSM.TaskToDo.FtpMsgToSend)
	  				{
	  					SMSUartTxState = FTPMsgWrite;
	  				}
	  				else if(GSM.TaskToDo.SmsMsgToSend && !GSM.TaskToDo.FtpMsgToSend)
	  				{
	  					SMSUartTxState = SMSMsgWrite;
	  				}

	  				break;
	  			}
	  		}
	  		else if(SMSUartTxState == Reset)
	  		{
	  			static uint8_t TaskState = 0;
	  			switch(TaskState)
	  			{
	  			case 0:
	  				inquiryTimeVar = 500;
	  				HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin, RESET);
	  				TaskState = 1;
	  				break;
	  			case 1:
	  				inquiryTimeVar = 5000;
	  				HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin, SET);
	  				TaskState = 2;
	  				break;
	  			case 2:
	  				inquiryTimeVar = INQUIRY_TIME;
	  				TaskState = 0;
	  				SMSUartTxState = Config;
	  				break;
	  			}
	  		}
	  		else if(SMSUartTxState == Start)
	  		{
	  			inquiryTimeVar = 2000;
	  			static uint8_t TaskState = 0;
	  			switch(TaskState)
	  			{
	  			case 0:
	  				UartSend("AT+CREG?\r\n");
	  				TaskState = 1;
	  				break;
	  			case 1:
	  				if(GSM.CRegStat == 1)
	  				{
	  					inquiryTimeVar = INQUIRY_TIME;
	  					TaskState = 0;
	  					SMSUartTxState = Control;
	  					GSM.ResetCounter++;
	  				}
	  				else
	  				{
	  					TaskState = 0;
	  				}
	  				break;
//	  			case 2:
//	  				UartSend("AT+CMGDA=\"DEL ALL\"\r\n");
//	  				TaskState = 0;
//	  				break;
	  			}
	  		}
	  		else if(SMSUartTxState == SMSMsgWrite)
	  		{
	  			static uint8_t TaskState = 0;
	  			static char ATcmdSMS[128];
	  			switch(TaskState)
	  			{
	  			case 0:
	  				sprintf(ATcmdSMS, "AT+CMGS=\"%s\"\r\n", GSM.SMSNumber);
//	  				UartSendWoRxCtrl("AT+CMGS=\"+48885447216\"\r\n");
	  				UartSendWoRxCtrl(ATcmdSMS);
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
	  				break;
	  			case 3:
	  				inquiryTimeVar = INQUIRY_TIME;
	  				TaskState = 0;
	  				GSM.TaskToDo.SmsMsgToSend = 0;
	  				SMSUartTxState = Control;
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
	  				UartSend("AT+CNMI=0,0,0,0,0\r\n");
	  				TaskState = 4;
	  				break;
	  			case 4:
	  				UartSend("AT+CIURC=0\r\n");
	  				TaskState = 5;
	  				break;
	  			case 5:
	  				UartSend("AT&W\r\n");
	  				TaskState = 0;
	  				SMSUartTxState = Start;
	  				break;
	  			}
	  		}
	  		else if(SMSUartTxState == FTPMsgWrite)
	  		{
	  			static uint8_t TaskState = 0;
				uint16_t MsgLen;
				static char ATcmdFtp[128];
	  			inquiryTimeVar = INQUIRY_TIME;
	  			switch(TaskState)
	  			{
	  			case 0:
	  				UartSend("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
	  				TaskState = 1;
	  				break;
	  			case 1:
	  				sprintf(ATcmdFtp, "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n", GSM.ConfigFlash.apn);
//	  				UartSend("AT+SAPBR=3,1,\"APN\",\"plus\"\r\n");
					UartSend(ATcmdFtp);
	  				TaskState = 2;
	  				break;
	  			case 2:
	  				UartSend("AT+SAPBR=1,1\r\n");
	  				inquiryTimeVar = 5000;
	  				TaskState = 3;
	  				break;
	  			case 3:
	  				if(GSM.ReceivedState == 1)
	  				{
	  					TaskState = 4;
	  				}
	  				else
	  				{
	  					TaskState = 0;
	  					UartSend("AT+SAPBR=0,1\r\n");
	  				}
	  				break;
	  			case 4:
	  				UartSend("AT+SAPBR=2,1\r\n");
	  				inquiryTimeVar = INQUIRY_TIME;
	  				TaskState = 5;
	  				break;
	  			case 5:
	  				UartSend("AT+FTPCID=1\r\n");
	  				TaskState = 6;
	  				break;
	  			case 6:
	  				sprintf(ATcmdFtp, "AT+FTPSERV=\"%s\"\r\n", GSM.ConfigFlash.server);
	  				UartSend(ATcmdFtp);
	  				TaskState = 7;
	  				break;
	  			case 7:
	  				sprintf(ATcmdFtp, "AT+FTPUN=\"%s\"\r\n", GSM.ConfigFlash.login);
	  				UartSend(ATcmdFtp);
	  				TaskState = 8;
	  				break;
	  			case 8:
	  				sprintf(ATcmdFtp, "AT+FTPPW=\"%s\"\r\n", GSM.ConfigFlash.password);
	  				UartSend(ATcmdFtp);
	  				TaskState = 9;
	  				break;
	  			case 9:
	  				sprintf(ATcmdFtp, "AT+FTPPUTNAME=\"Termo%s%.2u%.2u%.2u%.2u%.2u%.2u.txt\"\r\n",GSM.ConfigFlash.deviceNumber, year, month, day, hour, minute, second);
	  				UartSend(ATcmdFtp);
	  				TaskState = 10;
	  				break;
	  			case 10:
	  				sprintf(ATcmdFtp, "AT+FTPPUTPATH=\"%s\"\r\n", GSM.ConfigFlash.path);
	  				UartSend(ATcmdFtp);
	  				TaskState = 11;
	  				break;
	  			case 11:
	  				UartSend("AT+FTPPUT=1\r\n");
	  				inquiryTimeVar = 4000;
	  				TaskState = 12;
	  				break;
	  			case 12:
	  				if(GSM.FtpPut.status != 1)
	  				{
	  					TaskState = 0;
	  					break;
	  				}
	  				if(FTPMessageBoxRecordSwitch == 2)
	  				{
	  					MsgLen = strlen(FTPMessageBox1);
	  				}
	  				else if(FTPMessageBoxRecordSwitch == 1)
	  				{
	  					MsgLen = strlen(FTPMessageBox2);
	  				}
	  				sprintf(ATcmdFtp,"AT+FTPPUT=2,%u\r\n", MsgLen);
	  				UartSendWoRxCtrl(ATcmdFtp);
					TaskState = 13;
					break;
	  			case 13:
	  				if(FTPMessageBoxRecordSwitch == 2)
	  				{
	  					UartSendWoRxCtrl(FTPMessageBox1);
	  				}
	  				else if(FTPMessageBoxRecordSwitch == 1)
	  				{
	  					UartSendWoRxCtrl(FTPMessageBox2);
	  				}
	  				TaskState = 14;
	  				break;
	  			case 14:
	  				UartSend("AT+FTPPUT=2,0\r\n");
	  				TaskState = 15;
	  				break;
	  			case 15:
	  				UartSend("AT+SAPBR=0,1\r\n");
	  				TaskState = 0;
	  				GSM.TaskToDo.FtpMsgToSend = 0;
	  				SMSUartTxState = Control;
	  				inquiryTimeVar = INQUIRY_TIME;
	  				break;

	  			}

	  		}
	  	}
}

void CDC_ReveiveCallback(uint8_t *Buffer, uint8_t Length)
{
	if(Length > 0)
		{
			volatile uint8_t i = 0;
			while(i < Length)
			{
				if(Buffer[i] != '\r')
				{
					if (RB_OK == Ring_Buffer_Write(&USBConfBuffer, Buffer[i]))
					{
						if(Buffer[i] == ENDLINE)
						{
							USBLineCounter++;
						}
						i++;
					}
				}
				else i++;
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
