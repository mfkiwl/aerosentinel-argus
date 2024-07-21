/*
 * bme680_main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS_H/BME680/bme680_main.h"

#include "main.h" // Include the header that defines SPI2_CSB_Pin and SPI2_CSB_GPIO_Port

extern SPI_HandleTypeDef hspi2;
struct bme68x_dev bme680_dev;
struct bme68x_dev *dev_bme = &bme680_dev;
uint8_t bme680_dev_addr;
uint8_t BME_GTXBuffer[512], BME_GRXBuffer[512];


/**
  * @defgroup    BME680 SPI Platform Read/Write Group
  * @brief       This section provide a set of functions used to read on the ASM330LHH I2C Port
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

BME68X_INTF_RET_TYPE bme680_platform_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    //BME_GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    HAL_SPI_TransmitReceive(&hspi2, &reg_addr, reg_data, len+1, 1000); // timeout 1000msec;
    while(hspi2.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_SET); // NSS high
    //memcpy(reg_data, BME_GRXBuffer+1, len);

    return 0;
}

BME68X_INTF_RET_TYPE bme680_platform_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    //BME_GTXBuffer[0] = reg_addr & 0x7F;
    //memcpy(&BME_GTXBuffer[1], reg_data, len);

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    HAL_SPI_Transmit(&hspi2, &reg_addr, len+1, 1000); // send register address + write data
    while(hspi2.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_SET); // NSS high

    return 0;
}



/**
  * @}
  *
  */

void bme680_delay_func(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}


/**
  * @brief  BME680 Initialization Function
  *
  * @param  bme680_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
  *
  */
int8_t BME680_Init(){



	uint8_t chipid;
	int8_t rslt = BME68X_OK;

	dev_bme->intf = BME68X_SPI_INTF;

	dev_bme->read = bme680_platform_read;
	dev_bme->write = bme680_platform_write;
	dev_bme->intf_ptr = &bme680_dev_addr;;
	dev_bme->delay_us = bme680_delay_func;
	dev_bme->amb_temp = 25;
    bme680_delay_func(100000, dev_bme->intf_ptr);

    bme680_platform_read(BME68X_REG_CHIP_ID, &chipid, 1, dev_bme->intf_ptr);

    bme680_delay_func(100000, dev_bme->intf_ptr);
	/* Initialize bmi323. */
	rslt += bme68x_init(dev_bme);
	//printf("bmi323_init: %d \n",rslt);

	if (rslt != BME68X_OK)
	{
		printf("bme680_init() failed, error code: %d\r\n", rslt);
		return rslt;
	}
	else
	{
		printf("BME680 initialized successfully\r\n");
	}

	rslt += bme68x_get_regs(BME68X_REG_CHIP_ID, &chipid, 1, dev_bme);
	if (rslt != BME68X_OK)
	{
		printf("read chip ID failed, error code: %d\r\n", rslt);
		return rslt;
	}


    return rslt;
}
