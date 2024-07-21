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
#include "DRIVERS_H/BME680/bme68x.h"
#include "DRIVERS_H/BME680/bme68x_defs.h"

BME68X_INTF_RET_TYPE bme680_platform_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BME68X_INTF_RET_TYPE bme680_platform_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bme680_delay_func(uint32_t period, void *intf_ptr);

int8_t BME680_Init();

#endif /* INC_DRIVERS_H_BME680_BME680_MAIN_H_ */
