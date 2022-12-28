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

void UartLog(char * Message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*) Message, strlen(Message), 20);
}

void UartSend(char * Message)
{
	ReceivedState = 0;
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) Message, strlen(Message));
}
