/*
 * bme680_main.h
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_BME680_BME680_MAIN_H_
#define INC_DRIVERS_H_BME680_BME680_MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx.h"
#include <stdbool.h>
#include <string.h>

int8_t bme680_platform_read(SPI_HandleTypeDef *handle, uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bme680_read_wrapper(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bme680_platform_write(SPI_HandleTypeDef *handle, uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bme680_write_wrapper(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

bool BME680_Init();

#endif /* INC_DRIVERS_H_BME680_BME680_MAIN_H_ */
