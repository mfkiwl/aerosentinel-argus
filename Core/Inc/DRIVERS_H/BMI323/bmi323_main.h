/*
 * bmi323_main.h
 *
 *  Created on: Jul 15, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_BMI323_BMI323_MAIN_H_
#define INC_DRIVERS_H_BMI323_BMI323_MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx.h"
#include <stdbool.h>
#include <string.h>

int8_t bmi323_platform_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bmi323_platform_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

bool BMI323_Init();


#endif /* INC_DRIVERS_H_BMI323_BMI323_MAIN_H_ */
