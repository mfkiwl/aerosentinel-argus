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
int32_t asm330lhh_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint32_t errorCode;

    // Perform the I2C read operation using HAL_I2C_Mem_Read
    status = HAL_I2C_Mem_Read(&hi2c4, ASM330LHH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);

    // Check the status and print detailed error information
    if (status != HAL_OK) {
        // Get the error code
        errorCode = HAL_I2C_GetError(handle);
        printf("I2C Read Status: %d\n", status);
        printf("I2C Error Code: 0x%08lX\n", errorCode);

        // Print specific error details
        if (errorCode & HAL_I2C_ERROR_BERR) {
            printf("Error: Bus Error\n");
        }
        if (errorCode & HAL_I2C_ERROR_ARLO) {
            printf("Error: Arbitration Lost\n");
        }
        if (errorCode & HAL_I2C_ERROR_AF) {
            printf("Error: Acknowledge Failure\n");
        }
        if (errorCode & HAL_I2C_ERROR_OVR) {
            printf("Error: Overrun/Underrun\n");
        }
        if (errorCode & HAL_I2C_ERROR_DMA) {
            printf("Error: DMA Transfer Error\n");
        }
        if (errorCode & HAL_I2C_ERROR_TIMEOUT) {
            printf("Error: Timeout Error\n");
        }
    } else {
        printf("I2C Read Success: %x\n", *bufp);
    }

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
int32_t asm330lhh_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  // Perform the I2C write operation using HAL_I2C_Mem_Write
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c4, (ASM330LHH_I2C_ADD_L << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 100);
  // Return the appropriate value based on the HAL status
  return (status == HAL_OK) ? 0 : -1;
}

/**
  * @}
  *
  */

void asm330lhh_delay_msec(uint32_t msecs)
{
    HAL_Delay(msecs);
}


/**
  * @brief  ASM330LHH Initialization Function
  *
  * @param  asm330lhh_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
  *
  */
bool ASM330LHH_Init(){

	//Assign Read/Write functions to the device
	asm330lhh.read_reg = asm330lhh_platform_read;
	asm330lhh.write_reg = asm330lhh_platform_write;
	//Assign I2C Port of device
	asm330lhh.handle =  &hi2c4;
	asm330lhh.mdelay = asm330lhh_delay_msec;


	/* Check device ID */
		whoamI = 0;
		//asm330lhh_device_id_get(&asm330lhh, &whoamI);

//		if ( whoamI != ASM330LHH_ID ) {
//			printf("ASM330LHH ID Error!! Should be : %hhu \n",whoamI);
//			return 0;
//		}
		int32_t ret = asm330lhh_device_id_get(&asm330lhh, &whoamI);

			printf("%d \n", ret);
			return 0;

		//printf("ASM330LHH 6-Axis Automotive IMU Found!");

		//TODO -> Configure the device if recognised successfully.
		//return 1;


}


