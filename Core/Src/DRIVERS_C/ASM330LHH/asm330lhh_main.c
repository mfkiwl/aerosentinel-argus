/*
 * asm330lhh_main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS_H/ASM330LHH/asm330lhh_main.h"
#include "DRIVERS_H/ASM330LHH/asm330lhh_reg.h"




stmdev_ctx_t asm330lhh;
static uint8_t whoamI;

extern I2C_HandleTypeDef hi2c4;

/**
  * @defgroup    ASM330LH I2C Platform Read Group
  * @brief       This section provide a set of functions used to read on the ASM330LHH I2C Port
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
int32_t asm330lhh_platform_read(I2C_HandleTypeDef *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  // Perform the I2C read operation using HAL_I2C_Mem_Read
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, ASM330LHH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  // Return the appropriate value based on the HAL status
  return (status == HAL_OK) ? 0 : -1;
}


/**
  * @brief  I2C PLATFORM READING FUNCTION WRAPPER
  *
  * @param  handle  I2C Port (void parameter to avoid driver warning)
  * @param  reg     register to read
  * @param  bufp    pointer to buffer that store the data read(ptr)
  * @param  len     number of consecutive register to read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330lhh_read_wrapper(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    return asm330lhh_platform_read((I2C_HandleTypeDef *)handle, reg, bufp, len);
}

/**
  * @}
  *
  */

/**
  * @defgroup    ASM330LH I2C Platform Write Group
  * @brief       This section provide a set of functions used to write on the ASM330LHH I2C Port
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

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
int32_t asm330lhh_platform_write(I2C_HandleTypeDef *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  // Perform the I2C write operation using HAL_I2C_Mem_Write
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle, ASM330LHH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  // Return the appropriate value based on the HAL status
  return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  I2C PLATFORM WRITING FUNCTION WRAPPER
  *
  * @param  handle  I2C Port (void parameter to avoid driver warning)
  * @param  reg     register to read
  * @param  bufp    pointer to buffer that store the data read(ptr)
  * @param  len     number of consecutive register to read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330lhh_write_wrapper(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    return asm330lhh_platform_write((I2C_HandleTypeDef *)handle, reg, (uint8_t *)bufp, len);
}

/**
  * @}
  *
  */



/**
  * @brief  ASM330LHH Initialization Function
  *
  * @param  handle  I2C Port (void parameter to avoid driver warning)
  * @param  asm330lhh_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
  *
  */
bool ASM330LHH_Init(){

	//Assign Read/Write functions to the device
	asm330lhh.read_reg = asm330lhh_read_wrapper;
	asm330lhh.write_reg = asm330lhh_write_wrapper;
	//Assign I2C Port of device
	asm330lhh.handle =  &hi2c4;


	/* Check device ID */
		whoamI = 0;
		asm330lhh_device_id_get(&asm330lhh, &whoamI);

		if ( whoamI != ASM330LHH_ID ) {
			printf("ASM330LHH ID Error!!");
			return 0;
		}
		printf("ASM330LHH 6-Axis Automotive IMU Found!");

		//TODO -> Configure the device if recognised successfully.
		return 1;


}


