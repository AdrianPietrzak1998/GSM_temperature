/*
 * utils.c
 *
 *  Created on: 4 cze 2022
 *      Author: Adrian
 */

#include "main.h"
#include "usart.h"
#include "string.h"
#include "utils.h"
#include "tim.h"

void UartLog(char * Message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*) Message, strlen(Message), 20);
}

void UartSend(char * Message)
{
	GSM.ReceivedState = 0;
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) Message, strlen(Message));
	*Uart1isBusyPtr = 1;
}

void UartSendWoRxCtrl(char * Message)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) Message, strlen(Message));
	*Uart1isBusyPtr = 1;
}

void Temperature100ToString(int32_t temp, char *StringBuf)
{
	int16_t Decimal;
	uint16_t Fractial;

	Decimal = temp/100;

	if(temp>=0)
	{
		Fractial = temp - Decimal * 100;
	}
	else
	{
		Fractial = (temp * -1) - ((Decimal * -1) * 100);
	}

	sprintf(StringBuf, "%.2i.%.2u", Decimal, Fractial);
}
