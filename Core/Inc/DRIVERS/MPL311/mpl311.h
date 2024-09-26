/*
 * mpl311.h
 *
 *  Created on: Sep 26, 2024
 *      Author: Utilisateur
 */

#ifndef INC_DRIVERS_MPL311_MPL311_H_
#define INC_DRIVERS_MPL311_MPL311_H_

#include "stm32h7xx_hal.h" // Replace with your specific STM32 HAL header
#include <stdint.h>
#include <stdbool.h>

#define MPL3115A2_ADDRESS (0x60 << 1) ///< Default I2C address (shifted for STM32 HAL)

// MPL3115A2 registers
enum {
  MPL3115A2_REGISTER_STATUS = (0x00),
  MPL3115A2_REGISTER_PRESSURE_MSB = (0x01),
  MPL3115A2_REGISTER_PRESSURE_CSB = (0x02),
  MPL3115A2_REGISTER_PRESSURE_LSB = (0x03),
  MPL3115A2_REGISTER_TEMP_MSB = (0x04),
  MPL3115A2_REGISTER_TEMP_LSB = (0x05),
  MPL3115A2_REGISTER_DR_STATUS = (0x06),
  MPL3115A2_OUT_P_DELTA_MSB = (0x07),
  MPL3115A2_OUT_P_DELTA_CSB = (0x08),
  MPL3115A2_OUT_P_DELTA_LSB = (0x09),
  MPL3115A2_OUT_T_DELTA_MSB = (0x0A),
  MPL3115A2_OUT_T_DELTA_LSB = (0x0B),
  MPL3115A2_WHOAMI = (0x0C),
  MPL3115A2_BAR_IN_MSB = (0x14),
  MPL3115A2_BAR_IN_LSB = (0x15),
  MPL3115A2_OFF_H = (0x2D),
};

// MPL3115A2 status register bits
enum {
  MPL3115A2_REGISTER_STATUS_TDR = 0x02,
  MPL3115A2_REGISTER_STATUS_PDR = 0x04,
  MPL3115A2_REGISTER_STATUS_PTDR = 0x08,
};

// MPL3115A2 PT DATA register bits
enum {
  MPL3115A2_PT_DATA_CFG = 0x13,
  MPL3115A2_PT_DATA_CFG_TDEFE = 0x01,
  MPL3115A2_PT_DATA_CFG_PDEFE = 0x02,
  MPL3115A2_PT_DATA_CFG_DREM = 0x04,
};

// MPL3115A2 control registers
enum {
  MPL3115A2_CTRL_REG1 = (0x26),
  MPL3115A2_CTRL_REG2 = (0x27),
  MPL3115A2_CTRL_REG3 = (0x28),
  MPL3115A2_CTRL_REG4 = (0x29),
  MPL3115A2_CTRL_REG5 = (0x2A),
};

// MPL3115A2 control register bits
enum {
  MPL3115A2_CTRL_REG1_SBYB = 0x01,
  MPL3115A2_CTRL_REG1_OST = 0x02,
  MPL3115A2_CTRL_REG1_RST = 0x04,
  MPL3115A2_CTRL_REG1_RAW = 0x40,
  MPL3115A2_CTRL_REG1_ALT = 0x80,
  MPL3115A2_CTRL_REG1_BAR = 0x00,
};

// MPL3115A2 oversample values
enum {
  MPL3115A2_CTRL_REG1_OS1 = 0x00,
  MPL3115A2_CTRL_REG1_OS2 = 0x08,
  MPL3115A2_CTRL_REG1_OS4 = 0x10,
  MPL3115A2_CTRL_REG1_OS8 = 0x18,
  MPL3115A2_CTRL_REG1_OS16 = 0x20,
  MPL3115A2_CTRL_REG1_OS32 = 0x28,
  MPL3115A2_CTRL_REG1_OS64 = 0x30,
  MPL3115A2_CTRL_REG1_OS128 = 0x38,
};

// MPL3115A2 measurement modes
typedef enum {
  MPL3115A2_BAROMETER = 0,
  MPL3115A2_ALTIMETER,
} mpl3115a2_mode_t;

// MPL3115A2 measurement types
typedef enum {
  MPL3115A2_PRESSURE,
  MPL3115A2_ALTITUDE,
  MPL3115A2_TEMPERATURE,
} mpl3115a2_meas_t;

// Start conversion register
#define MPL3115A2_REGISTER_STARTCONVERSION (0x12)

typedef struct {
  I2C_HandleTypeDef *hi2c; ///< Pointer to I2C handle
  mpl3115a2_mode_t currentMode;

  union {
    struct {
      uint8_t SBYB : 1;
      uint8_t OST : 1;
      uint8_t RST : 1;
      uint8_t OS : 3;
      uint8_t RAW : 1;
      uint8_t ALT : 1;
    } bit;
    uint8_t reg;
  } ctrl_reg1;

} MPL3115A2;


typedef struct {
	float pressure;
	float temperature;
	float altitude;
}Altimeter_3_Axis;

#ifdef __cplusplus
extern "C" {
#endif

int8_t MPL3115A2_Init();
float MPL3115A2_GetPressure();
float MPL3115A2_GetAltitude();
int8_t MPL3115A2_GetAltitudeOffset();
void MPL3115A2_SetAltitudeOffset(int8_t offset);
float MPL3115A2_GetTemperature();
void MPL3115A2_SetSeaPressure(float SLP);
void MPL3115A2_SetMode(mpl3115a2_mode_t mode);
void MPL3115A2_StartOneShot();
bool MPL3115A2_ConversionComplete();
float MPL3115A2_GetLastConversionResults(mpl3115a2_meas_t value);
Altimeter_3_Axis mpl311_read_data();
void mpl311_print_altimetry(Altimeter_3_Axis *data);

#ifdef __cplusplus
}
#endif

#endif /* INC_DRIVERS_MPL311_MPL311_H_ */
