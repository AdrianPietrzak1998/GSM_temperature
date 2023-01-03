/*
 * 1wire.h
 *
 *  Created on: Jan 2, 2023
 *      Author: Adrian
 */

#ifndef INC_1WIRE_H_
#define INC_1WIRE_H_

#include "stm32f1xx.h"

#define UART    huart2
#define USARTn   USART2

HAL_StatusTypeDef wire_init(void);
HAL_StatusTypeDef wire_reset(void);
uint8_t wire_read(void);
void wire_write(uint8_t byte);
uint8_t wire_crc(const uint8_t* data, int len);

#endif /* INC_1WIRE_H_ */
