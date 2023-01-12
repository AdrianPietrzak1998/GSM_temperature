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
	else if(strcmp("log", (char*)DataToParse) == 0)
	{
		sprintf(SMSMessage, "CSQ: %.1f", GSM.SignalQuality);
		SMSUartTxState = SMSMsgWrite;
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
