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

	SignalQuality = atof(ParsePointer);
}

static void Parser_ParseCREG(void)
{
	char * ParsePointer = strtok(NULL, ",");
	CRegN = atoi(ParsePointer);
	ParsePointer = strtok(NULL, ",");
	CRegStat = atoi(ParsePointer);
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
		ReceivedState = 1;
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
		else if(strcmp("Test2", ParsePointer) == 0)
		{
			SMSUartTxState = SMSMsgWrite;
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
