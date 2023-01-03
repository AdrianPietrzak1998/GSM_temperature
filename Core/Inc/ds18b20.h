/*
 * ds18b20.h
 *
 *  Created on: Jan 2, 2023
 *      Author: Adrian
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "stm32f1xx.h"

#define DS18B20_ROM_CODE_SIZE		8
HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code);
HAL_StatusTypeDef ds18b20_start_measure(const uint8_t* rom_code); //rom_code or NULL
float ds18b20_get_temp(const uint8_t* rom_code); //rom_code or NULL
int32_t ds18b20_get_temp_wo_fp(const uint8_t* rom_code);


#endif /* INC_DS18B20_H_ */
