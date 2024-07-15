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


s8 bno055_platform_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 bno055_platform_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

void bno055_delay_msec(u32 msecs);

bool BNO055_Init();

#endif /* INC_DRIVERS_H_BNO055_BNO055_MAIN_H_ */
