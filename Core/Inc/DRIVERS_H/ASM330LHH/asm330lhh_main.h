/*
 * asm330lhh_main.h
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_ASM330LHH_ASM330LHH_MAIN_H_
#define INC_DRIVERS_H_ASM330LHH_ASM330LHH_MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx.h"
#include <stdbool.h>


int32_t asm330lhh_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t asm330lhh_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

bool ASM330LHH_Init();

#endif /* INC_DRIVERS_H_ASM330LHH_ASM330LHH_MAIN_H_ */
