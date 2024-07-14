/*
 * bno055_main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS_H/BNO055/bno055_main.h"


struct bno055_t bno055_dev;

extern I2C_HandleTypeDef hi2c2;



/**
  * @defgroup    ASM330LH I2C Platform Read Group
  * @brief       This section provide a set of functions used to read on the bno055 I2C Port
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
s8 bno055_platform_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  // Perform the I2C read operation using HAL_I2C_Mem_Read
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 100);
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
s8 bno055_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    return bno055_platform_read(dev_addr, reg_addr, reg_data, cnt);
}

/**
  * @}
  *
  */

/**
  * @defgroup    ASM330LH I2C Platform Write Group
  * @brief       This section provide a set of functions used to write on the bno055 I2C Port
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
s8 bno055_platform_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  // Perform the I2C write operation using HAL_I2C_Mem_Write
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 100);
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
s8 bno055_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    return bno055_platform_write(dev_addr, reg_addr, reg_data, cnt);
}

/**
  * @}
  *
  */


/**
  * @brief  BNO055 Delay Function
  *
  * @retval         void
  *
  */
void bno055_delay_msec(u32 msecs)
{
    HAL_Delay(msecs);
}


/**
  * @brief  BNO055 Initialization Function
  *
  * @param  		bno055_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
  *
  */
bool BNO055_Init(){
	bno055_dev.bus_write = bno055_write_wrapper;
	bno055_dev.bus_read = bno055_read_wrapper;
	bno055_dev.delay_msec = bno055_delay_msec;
	bno055_dev.dev_addr = BNO055_I2C_ADDR1;

	int8_t bno055_status = bno055_init(&bno055_dev);


	return !bno055_status;
}



