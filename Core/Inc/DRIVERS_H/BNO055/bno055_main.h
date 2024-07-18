/*
 * bno055_main.h
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_BNO055_BNO055_MAIN_H_
#define INC_DRIVERS_H_BNO055_BNO055_MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx.h"
#include <stdbool.h>
#include "DRIVERS_H/BNO055/bno055.h"

#define BUS_TIMEOUT             1000

signed char bno055_platform_read(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char ReadNumbr);
signed char bno055_platform_write(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char WriteNumbr);

void bno055_delay_msec(u32 msecs);

int8_t BNO055_Init();

#endif /* INC_DRIVERS_H_BNO055_BNO055_MAIN_H_ */
