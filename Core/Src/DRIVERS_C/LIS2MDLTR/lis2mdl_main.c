/*
 * lis2mdl_main.c
 *
 *  Created on: Jul 15, 2024
 *      Author: Yassine DEHHANI
 */



#include "DRIVERS_H/LIS2MDLTR/lis2mdl_main.h"
#include "DRIVERS_H/LIS2MDLTR/lis2mdl_reg.h"


stmdev_ctx_t lis2mdltr;
static uint8_t whoamI;

extern I2C_HandleTypeDef hi2c1;

/**
  * @defgroup    LIS2MDLTR I2C Platform Read Group
  * @brief       This section provide a set of functions used to read on the lis2mdltr I2C Port
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */
/**

  * @brief  I2C PLATFORM READING FUNCTION
  *
  * @param  handle  I2C Port
  * @param  reg     register to read
  * @param  bufp    pointer to buffer that store the data read(ptr)
  * @param  len     number of consecutive register to read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdltr_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  // Perform the I2C read operation using HAL_I2C_Mem_Read
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read((I2C_HandleTypeDef *)handle, LIS2MDL_I2C_ADD << 1, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  // Return the appropriate value based on the HAL status
  return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  I2C PLATFORM WRITING FUNCTION
  *
  * @param  handle  I2C Port
  * @param  reg     register to read
  * @param  bufp    pointer to buffer that store the data read(ptr)
  * @param  len     number of consecutive register to read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */


int32_t lis2mdltr_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  // Perform the I2C write operation using HAL_I2C_Mem_Write
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write((I2C_HandleTypeDef *)handle, LIS2MDL_I2C_ADD << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 100);
  // Return the appropriate value based on the HAL status
  return (status == HAL_OK) ? 0 : -1;
}

/**
  * @}
  *
  */



/**
  * @brief  lis2mdltr Initialization Function
  *
  * @param  lis2mdltr_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
  *
  */
bool LIS2MDLTR_Init(){

	//Assign Read/Write functions to the device
	lis2mdltr.read_reg = lis2mdltr_platform_read;
	lis2mdltr.write_reg = lis2mdltr_platform_write;
	//Assign I2C Port of device
	lis2mdltr.handle =  &hi2c1;


	/* Check device ID */
		whoamI = 0;
		lis2mdl_device_id_get(&lis2mdltr, &whoamI);

		if ( whoamI != LIS2MDL_ID ) {
			printf("LIS2MDLTR ID Error!!");
			printf("Got ID : 0x%02X \n", whoamI);
			return 0;
		}
		printf("LIS2MDLTR High Performance 3-Axis Magnetometer Found!");

		//TODO -> Configure the device if recognised successfully.
		return 1;


}
