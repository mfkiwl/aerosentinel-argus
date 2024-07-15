/*
 * bme680_main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS_H/BME680/bme680_main.h"
#include "DRIVERS_H/BME680/bme68x.h"
#include "DRIVERS_H/BME680/bme68x_defs.h"

extern SPI_HandleTypeDef hspi2;
struct bme68x_dev bme680;


/**
  * @defgroup    BME680 SPI Platform Read/Write Group
  * @brief       This section provide a set of functions used to read on the ASM330LHH I2C Port
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  SPI PLATFORM READING FUNCTION
  *
  * @param  handle       SPI Port
  * @param  reg_addr     register to read
  * @param  reg_data     pointer to buffer that store the data read(ptr)
  * @param  length       number of consecutive register to read
  * @param  intf_ptr     interface pointer
  * @retval              interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t bme680_platform_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    HAL_StatusTypeDef status;

    status = HAL_SPI_Transmit((SPI_HandleTypeDef *)intf_ptr, &reg_addr, 1, 100);
    if (status != HAL_OK) return -1;

    status = HAL_SPI_Receive((SPI_HandleTypeDef *)intf_ptr, reg_data, length, 100);
    return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  SPI PLATFORM WRITING FUNCTION
  *
  * @param  handle       SPI Port
  * @param  reg_addr     register to read
  * @param  reg_data     pointer to buffer that store the data read(ptr)
  * @param  length       number of consecutive register to read
  * @param  intf_ptr     interface pointer
  * @retval              interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t bme680_platform_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    uint8_t tx_buffer[length + 1];
    tx_buffer[0] = reg_addr;
    memcpy(&tx_buffer[1], reg_data, length);

    HAL_StatusTypeDef status = HAL_SPI_Transmit((SPI_HandleTypeDef *)intf_ptr, tx_buffer, length + 1, 100);
    return (status == HAL_OK) ? 0 : -1;
}

/**
  * @}
  *
  */


/**
  * @brief  BME680 Initialization Function
  *
  * @param  bme680_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
  *
  */
bool BME680_Init(){

	//Device interface selection and assignment
    bme680.intf_ptr = &hspi2;
    bme680.intf = BME68X_SPI_INTF;

	//Assign Read/Write functions to the device
    bme680.read = bme680_platform_read;
    bme680.write = bme680_platform_write;

    int8_t bme680_status = bme68x_init(&bme680);


    return !bme680_status;
}
