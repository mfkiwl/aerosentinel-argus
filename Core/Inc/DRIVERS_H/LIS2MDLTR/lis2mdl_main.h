/*
 * lis2mdl_main.h
 *
 *  Created on: Jul 15, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_LIS2MDLTR_LIS2MDL_MAIN_H_
#define INC_DRIVERS_H_LIS2MDLTR_LIS2MDL_MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx.h"
#include <stdbool.h>
#include <string.h>

int32_t lis2mdltr_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t lis2mdltr_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

bool LIS2MDLTR_Init();

#endif /* INC_DRIVERS_H_LIS2MDLTR_LIS2MDL_MAIN_H_ */
