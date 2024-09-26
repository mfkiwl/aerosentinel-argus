/*
 * mpl311.c
 *
 *  Created on: Sep 26, 2024
 *      Author: Yassine DEHHANI
 */


#include "DRIVERS/MPL311/mpl311.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

MPL3115A2 mpl311_dev;
extern I2C_HandleTypeDef hi2c2;

int8_t MPL3115A2_Init() {
  mpl311_dev.hi2c = &hi2c2;

  // Read WHOAMI register
  uint8_t whoami = 0;
  if (HAL_I2C_Mem_Read(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_WHOAMI,
                       I2C_MEMADD_SIZE_8BIT, &whoami, 1, HAL_MAX_DELAY) != HAL_OK) {
    return -1; // I2C read error
  }
  if (whoami != 0xC4) {
    return -2; // WHOAMI mismatch error
  }

  HAL_Delay(10);

//  uint8_t ctrl_reg1 = 0x00; // Clear the SBYB bit to enter STANDBY mode
//  if (HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1,
//                        I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY) != HAL_OK) {
//    return -3; // I2C write error
//  }
//
//  HAL_Delay(10); // Add a small delay to allow the sensor to switch to STANDBY mode
//
//  // Software reset
//  ctrl_reg1 = MPL3115A2_CTRL_REG1_RST;
//  if (HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1,
//                        I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY) != HAL_OK) {
//    return -3; // I2C write error
//  }
//
//  // Wait for reset to complete
//  do {
//    if (HAL_I2C_Mem_Read(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1,
//                         I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY) != HAL_OK) {
//      return -4; // I2C read error during reset
//    }
//    HAL_Delay(10);
//  } while (ctrl_reg1 & MPL3115A2_CTRL_REG1_RST);

  // Set oversampling and altitude mode
  mpl311_dev.currentMode = MPL3115A2_ALTIMETER;
  mpl311_dev.ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
  if (HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1,
                        I2C_MEMADD_SIZE_8BIT, &(mpl311_dev.ctrl_reg1.reg), 1, HAL_MAX_DELAY) != HAL_OK) {
    return -5; // I2C write error
  }

  // Enable data ready events for pressure/altitude and temperature
  uint8_t pt_data_cfg = MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_PDEFE |
                        MPL3115A2_PT_DATA_CFG_DREM;
  if (HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_PT_DATA_CFG,
                        I2C_MEMADD_SIZE_8BIT, &pt_data_cfg, 1, HAL_MAX_DELAY) != HAL_OK) {
    return -6; // I2C write error
  }

  return 0; // Initialization successful
}

float MPL3115A2_GetPressure() {
  if (mpl311_dev.currentMode != MPL3115A2_BAROMETER)
    MPL3115A2_SetMode(MPL3115A2_BAROMETER);
  MPL3115A2_StartOneShot(mpl311_dev);
  while (!MPL3115A2_ConversionComplete(mpl311_dev))
    HAL_Delay(10);
  return MPL3115A2_GetLastConversionResults(MPL3115A2_PRESSURE);
}

float MPL3115A2_GetAltitude() {
  if (mpl311_dev.currentMode != MPL3115A2_ALTIMETER)
    MPL3115A2_SetMode(MPL3115A2_ALTIMETER);
  MPL3115A2_StartOneShot(mpl311_dev);
  while (!MPL3115A2_ConversionComplete(mpl311_dev))
    HAL_Delay(10);
  return MPL3115A2_GetLastConversionResults(MPL3115A2_ALTITUDE);
}

int8_t MPL3115A2_GetAltitudeOffset() {
  uint8_t offset;
  if (HAL_I2C_Mem_Read(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_OFF_H, I2C_MEMADD_SIZE_8BIT, &offset, 1, HAL_MAX_DELAY) != HAL_OK) {
    return 0;
  }
  return (int8_t)offset;
}

void MPL3115A2_SetAltitudeOffset(int8_t offset) {
  uint8_t data = (uint8_t)offset;
  HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_OFF_H, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

float MPL3115A2_GetTemperature() {
  MPL3115A2_StartOneShot(mpl311_dev);
  while (!MPL3115A2_ConversionComplete(mpl311_dev))
    HAL_Delay(10);
  return MPL3115A2_GetLastConversionResults(MPL3115A2_TEMPERATURE);
}

void MPL3115A2_SetSeaPressure(float SLP) {
  uint16_t bar = (uint16_t)(SLP * 50);
  uint8_t buffer[2];
  buffer[0] = bar >> 8;
  buffer[1] = bar & 0xFF;
  HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_BAR_IN_MSB, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);
}

void MPL3115A2_SetMode(mpl3115a2_mode_t mode) {
  uint8_t ctrl_reg1;
  // Read current CTRL_REG1
  if (HAL_I2C_Mem_Read(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY) != HAL_OK) {
    return;
  }
  mpl311_dev.ctrl_reg1.reg = ctrl_reg1;
  mpl311_dev.ctrl_reg1.bit.ALT = mode;
  // Write back CTRL_REG1
  ctrl_reg1 = mpl311_dev.ctrl_reg1.reg;
  HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY);
  mpl311_dev.currentMode = mode;
}

void MPL3115A2_StartOneShot() {
  uint8_t ctrl_reg1;
  // Wait for OST bit to clear
  do {
    if (HAL_I2C_Mem_Read(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY) != HAL_OK) {
      return;
    }
    HAL_Delay(10);
  } while (ctrl_reg1 & MPL3115A2_CTRL_REG1_OST);
  // Set OST bit to initiate measurement
  ctrl_reg1 |= MPL3115A2_CTRL_REG1_OST;
  HAL_I2C_Mem_Write(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY);
}

bool MPL3115A2_ConversionComplete() {
  uint8_t status;
  if (HAL_I2C_Mem_Read(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_REGISTER_STATUS, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY) != HAL_OK) {
    return false;
  }
  return (status & MPL3115A2_REGISTER_STATUS_PTDR) != 0;
}

float MPL3115A2_GetLastConversionResults(mpl3115a2_meas_t value) {
  uint8_t buffer[5];
  // Read data starting from MPL3115A2_REGISTER_PRESSURE_MSB
  if (HAL_I2C_Mem_Read(mpl311_dev.hi2c, MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB, I2C_MEMADD_SIZE_8BIT, buffer, 5, HAL_MAX_DELAY) != HAL_OK) {
    return 0.0;
  }

  switch (value) {
  case MPL3115A2_PRESSURE:
  {
    uint32_t pressure;
    pressure = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    return ((float)pressure) / 6400.0;
  }
  case MPL3115A2_ALTITUDE:
  {
    int32_t alt;
    // Sign-extend the 20-bit altitude data
    alt = ((int32_t)buffer[0] << 24) | ((int32_t)buffer[1] << 16) | ((int32_t)buffer[2] << 8);
    alt >>= 8; // Shift back to get correct sign
    return ((float)alt) / 65536.0;
  }
  case MPL3115A2_TEMPERATURE:
  default:
  {
    int16_t t;
    t = ((int16_t)buffer[3] << 8) | buffer[4];
    return ((float)t) / 256.0;
  }
  }
}

Altimeter_3_Axis mpl311_read_data(){
	Altimeter_3_Axis data;

	data.pressure = MPL3115A2_GetPressure();

	data.temperature = MPL3115A2_GetTemperature();

	data.altitude = MPL3115A2_GetAltitude();

	return data;
}

void mpl311_print_altimetry(Altimeter_3_Axis *data){
	printf("MPL311 Altimeter: \n");
	printf("Pressure: %.3f Pa, Temperature: %.3f degC, Altitude: %.3f meters \n", data->pressure, data->temperature, data->altitude);
    printf("----- \n");
}
