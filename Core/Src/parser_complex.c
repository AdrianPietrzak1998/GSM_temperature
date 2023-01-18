/*
 * parser_complex.c
 *
 *  Created on: 4 cze 2022
 *      Author: Adrian
 */

#include "main.h"
#include "ring_buffer.h"
#include "utils.h"
#include "parser_complex.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "FLASH_PAGE_F1.h"


void Parser_TakeLine(RingBuffer_t *Buff, uint8_t *Destination)
{
	  uint8_t i = 0;
	  uint8_t tmp = 0;
	do
	{
		 Ring_Buffer_Read(Buff, &tmp);
		 if(tmp == ENDLINE)
			{
			 Destination[i] = 0;
			}
		else
			{
			Destination[i] = tmp;
			}

			i++;

	} while(tmp != ENDLINE);
}

//CSQ


static void Parser_ParseCSQ(void)
{
	char * ParsePointer = strtok(NULL, ",");

	GSM.SignalQuality = atof(ParsePointer);
}

static void Parser_ParseCREG(void)
{
	char * ParsePointer = strtok(NULL, ",");
	GSM.CRegN = atoi(ParsePointer);
	ParsePointer = strtok(NULL, ",");
	GSM.CRegStat = atoi(ParsePointer);
}

static void Parser_ParseCCLK(void)
{
	char * ParsePointer = strtok(NULL, "/");
	year = atoi(ParsePointer+1);
	ParsePointer = strtok(NULL, "/");
	month = atoi(ParsePointer);
	ParsePointer = strtok(NULL, ",");
	day = atoi(ParsePointer);
	ParsePointer = strtok(NULL, ":");
	hour = atoi(ParsePointer);
	ParsePointer = strtok(NULL, ":");
	minute = atoi(ParsePointer);
	ParsePointer = strtok(NULL, "+");
	second = atoi(ParsePointer);
}

static void Parser_ParseFTPPUT(void)
{
	char * ParsePointer = strtok(NULL, ",");
	GSM.FtpPut.mode = atoi(ParsePointer);
	ParsePointer = strtok(NULL, ",");
	if(GSM.FtpPut.mode == 1)
	{
		GSM.FtpPut.status = atoi(ParsePointer);
		if(GSM.FtpPut.status == 1)
		{
			ParsePointer = strtok(NULL, ",");
			GSM.FtpPut.maxLength = atoi(ParsePointer);
		}
	}
	else if(GSM.FtpPut.mode == 2)
	{
		GSM.FtpPut.CnfLength = atoi(ParsePointer);
	}
}

static void Parser_ParseCMGL(void)
{
	char *ParsePointer = strtok(NULL, "\"");
	ParsePointer = strtok(NULL, "\"");
	ParsePointer = strtok(NULL, ",") +1;
	strtok(ParsePointer, "\"");
	strcpy(GSM.SMSNumber,ParsePointer);
}


void Parser_parse(uint8_t * DataToParse)
{

	if(strcmp("OK", (char*)DataToParse) == 0)
	{
		GSM.ReceivedState = 1;
	}
	else if(strcmp("save", (char*)DataToParse) == 0)
	{
		Flash_Write_Data(0x0801FC00, GSM.FlashBuff, 128);
	}
	else if(strcmp("Log", (char*)DataToParse) == 0)
	{
		sprintf(SMSMessage, "CSQ:%.1f\nERROR:%d\n%d\nAPN:%s\n%s\n%s", GSM.SignalQuality, GSM.ErrorCounter, GSM.ResetCounter, GSM.ConfigFlash.apn, GSM.ConfigFlash.path, GSM.ConfigFlash.server);
		GSM.TaskToDo.SmsMsgToSend = 1;
	}
	else if(strcmp("Temp", (char*)DataToParse) == 0)
	{
		char TemperatureString[7];
		Temperature100ToString(temperature, TemperatureString);
		sprintf(SMSMessage, "Temperature: %s", TemperatureString);
		GSM.TaskToDo.SmsMsgToSend = 1;
	}
	else if(strcmp("ResetGsm", (char*)DataToParse) == 0)
	{
		SMSUartTxState = Reset;

	}
	else if(strcmp("ResetUc", (char*)DataToParse) == 0)
	{
		HAL_NVIC_SystemReset();
	}
	else if(strcmp("ERROR", (char*)DataToParse) == 0)
	{
		GSM.ErrorCounter++;
	}
	else
	{
		char * ParsePointer = strtok((char*)DataToParse, " ");

		if(strcmp("+CSQ:", ParsePointer) == 0)
		{
			Parser_ParseCSQ();
		}
		else if(strcmp("+CREG:", ParsePointer) == 0)
		{
			Parser_ParseCREG();
		}
		else if(strcmp("+CCLK:", ParsePointer) == 0)
		{
			Parser_ParseCCLK();
		}
		else if(strcmp("+FTPPUT:", ParsePointer) == 0)
		{
			Parser_ParseFTPPUT();
		}
		else if(strcmp("+CMGL:", ParsePointer) == 0)
		{
			Parser_ParseCMGL();
		}
		else if(strcmp("login:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.login, ParsePointer);
		}
		else if(strcmp("password:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.password, ParsePointer);
		}
		else if(strcmp("server:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.server, ParsePointer);
		}
		else if(strcmp("path:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.path, ParsePointer);
		}
		else if(strcmp("device:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.deviceNumber, ParsePointer);
		}
		else if(strcmp("number1:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.number1, ParsePointer);
		}
		else if(strcmp("number2:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.number2, ParsePointer);
		}
		else if(strcmp("apn:", ParsePointer) == 0)
		{
			ParsePointer = strtok(NULL, "\r");
			strcpy(GSM.ConfigFlash.apn, ParsePointer);
		}
	}



//	  if(strcmp("LED_ON", (char*)DataToParse) == 0)
//	  {
//		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//		  UartLog("led_on\n\r");
//	  }
//	  else if(strcmp("LED_OFF", (char*)DataToParse) == 0)
//	  {
//	  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//	  	UartLog("led_off\n\r");
//	  }


}
