/*
 * 1wire.c
 *
 *  Created on: Jan 2, 2023
 *      Author: Adrian
 */

#include "1wire.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"



static void set_baudrate(uint32_t baudrate)
{
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = baudrate;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


HAL_StatusTypeDef wire_reset(void)
{
	uint8_t data_out = 0xF0;
	uint8_t data_in = 0;

	set_baudrate(9600);
	HAL_UART_Transmit(&UART, &data_out, 1, HAL_MAX_DELAY);
	HAL_UART_Receive(&UART, &data_in, 1, HAL_MAX_DELAY);
	set_baudrate(115200);

	if (data_in != 0xF0)
		return HAL_OK;
	else
		return HAL_ERROR;
}

static int read_bit(void)
{
	uint8_t data_out = 0xff;
	uint8_t data_in = 0x0;
	data_in = 0x0;
	if(USARTn -> SR & (1<<5))
	{
		USARTn -> DR;
	}
	HAL_UART_Transmit(&UART, &data_out, 1, HAL_MAX_DELAY);
	HAL_UART_Receive(&UART, &data_in, 1, HAL_MAX_DELAY);

	return data_in & 0x01;
}

uint8_t wire_read(void)
{
  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++)
  {
    value >>= 1;
    if (read_bit())
      value |= 0x80;
  }
  return value;
}

static void write_bit(int value)
{
  if (value)
  {
    uint8_t data_out = 0xff;
    HAL_UART_Transmit(&UART, &data_out, 1, HAL_MAX_DELAY);
  }
  else
  {
    uint8_t data_out = 0x0;
    HAL_UART_Transmit(&UART, &data_out, 1, HAL_MAX_DELAY);
  }
}

void wire_write(uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    write_bit(byte & 0x01);
    byte >>= 1;
  }
}

static uint8_t byte_crc(uint8_t crc, uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    uint8_t b = crc ^ byte;
    crc >>= 1;
    if (b & 0x01)
      crc ^= 0x8c;
    byte >>= 1;
  }
  return crc;
}

uint8_t wire_crc(const uint8_t* data, int len)
{
  int i;
    uint8_t crc = 0;

    for (i = 0; i < len; i++)
      crc = byte_crc(crc, data[i]);

    return crc;
}

