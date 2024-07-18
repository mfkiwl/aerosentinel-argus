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
#include <math.h>

#include "DRIVERS_H/BMI323/bmi3.h"
#include "DRIVERS_H/BMI323/bmi323.h"
#include "DRIVERS_H/BMI323/bmi3_defs.h"

//#include "stm32h7xx_hal.h"
typedef struct {
    float acceleration[3]; // X, Y, Z acceleration
    float gyroscope[3];    // X, Y, Z gyroscope
} IMU_6_Axis_Data;

/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(32)
/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
int8_t Open_BMI323_ACC();
int8_t Close_BMI323_ACC();
int8_t Open_BMI323_GYRO();
int8_t Close_BMI323_GYRO();
int8_t Open_BMI323_FIFO();
int8_t Close_BMI323_FIFO();
int8_t bmi3_interface_init(struct bmi3_dev *bmi, uint8_t intf);
int8_t Init_BMI323();
void BMI323_ReadData();
IMU_6_Axis_Data bmi323_data_poll();

int8_t bmi323_platform_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bmi323_platform_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);



#endif /* INC_DRIVERS_H_BMI323_BMI323_MAIN_H_ */
