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
struct bme68x_conf bme680_config;
struct bme68x_heatr_conf heater_config;
uint8_t bme680_dev_addr;
uint8_t BME_GTXBuffer[512], BME_GRXBuffer[512];
uint8_t n_fields;
uint32_t del_period;

struct bme68x_data temp_data;

/**
  * @defgroup    BME680 SPI Platform Read/Write Group
  * @brief       This section provide a set of functions used to read on the ASM330LHH I2C Port
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */


int8_t bme680_platform_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	BME_GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    HAL_SPI_TransmitReceive(&hspi2, BME_GTXBuffer, BME_GRXBuffer, length+1, 1000); // timeout 1000msec;
    while(hspi2.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_SET); // NSS High
    memcpy(reg_data, BME_GRXBuffer+1, length);

    return 0;
}

int8_t bme680_platform_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	BME_GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&BME_GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    HAL_SPI_Transmit(&hspi2, BME_GTXBuffer, length+1, 1000); // send register address + write data
    while(hspi2.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_SET); // NSS High

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
	//HAL_Delay(period/1000);
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
	dev_bme->intf_ptr = &bme680_dev_addr;
	dev_bme->delay_us = bme680_delay_func;
	dev_bme->amb_temp = 25;
    bme680_delay_func(100000, dev_bme->intf_ptr);

    bme680_platform_read(BME68X_REG_CHIP_ID, &chipid, 1, dev_bme->intf_ptr);

    bme680_delay_func(100000, dev_bme->intf_ptr);
	/* Initialize bmi323. */
	rslt += bme68x_init(dev_bme);

	if (rslt != BME68X_OK)
	{
		//printf("bme680_init() failed, error code: %d\r\n", rslt);
		return rslt;
	}
	else
	{
		//printf("BME680 initialized successfully\r\n");
	}

	rslt += bme68x_get_regs(BME68X_REG_CHIP_ID, &chipid, 1, dev_bme);
	if (rslt != BME68X_OK)
	{
		//printf("read chip ID failed, error code: %d\r\n", rslt);
		return rslt;
	}

	bme680_config.os_hum = BME68X_OS_2X;
	bme680_config.os_pres = BME68X_OS_4X;
	bme680_config.os_temp = BME68X_OS_8X;
	bme680_config.filter = BME68X_FILTER_SIZE_3;

	rslt += bme68x_set_conf(&bme680_config, dev_bme);

	heater_config.enable = 1;
	heater_config.heatr_temp = 320; //Target temperature in degrees Celsius
	heater_config.heatr_dur = 150; // Heating duration in milliseconds

	rslt += bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heater_config, dev_bme);




    return rslt;
}


Barometer_4_Axis bme680_data_poll(){
	Barometer_4_Axis data = {0};

	bme68x_set_op_mode(BME68X_FORCED_MODE, dev_bme);

	/* Calculate delay period in microseconds */
	del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme680_config, dev_bme) + (heater_config.heatr_dur * 1000);
	dev_bme->delay_us(del_period, dev_bme->intf_ptr);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &temp_data, &n_fields, dev_bme);

	//int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &temp_data, (uint8_t *)4, dev_bme);

	if(rslt == BME68X_OK){
		data.temperature = temp_data.temperature;
		data.pressure = temp_data.pressure;
		data.humidity = temp_data.humidity;
		data.gas_resistance = temp_data.gas_resistance;
	}
	return data;
}

void bme680_print_barometer_data(Barometer_4_Axis *data){
	printf("BME680 Barometer: \n");
	printf("Pressure: %f Pa, Temperature: %f degC, Humidity: %f , Gas Res: %f  Ohms\n", data->pressure, data->temperature, data->humidity, data->gas_resistance);
    printf("----- \n");
}

