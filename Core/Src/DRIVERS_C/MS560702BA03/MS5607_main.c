/*
 * MS5607_main.c
 *
 *  Created on: Jul 15, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS_H/MS560702BA03/MS5607_main.h"
#include "DRIVERS_H/MS560702BA03/MS5607.h"

extern SPI_HandleTypeDef hspi4;

extern GPIO_TypeDef *SPI4_CS_GPIO_Port;
extern uint16_t SPI4_CS_Pin;

bool MS560702BA03_Init(){

	MS5607StateTypeDef MS5607_code = MS5607_Init(&hspi4, SPI4_CS_GPIO_Port, SPI4_CS_Pin);
	return (MS5607_code == MS5607_STATE_FAILED) ? 0 : 1;
}
