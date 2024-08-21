#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "stm32h7xx.h"
#include "DRIVERS_H/BMI323/common_porting.h"
#include "main.h"

uint8_t GTXBuffer[512], GRXBuffer[2048];

extern SPI_HandleTypeDef hspi1;


void bst_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 64; i++)
		{
			;
		}
	}
}



#if defined(USE_BOSCH_SENSOR_API)


int8_t SensorAPI_SPIx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(SPI1_CSB_GPIO_Port, SPI1_CSB_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    HAL_SPI_TransmitReceive(&hspi1, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(hspi1.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI1_CSB_GPIO_Port, SPI1_CSB_Pin, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}

int8_t SensorAPI_SPIx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(SPI1_CSB_GPIO_Port, SPI1_CSB_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    HAL_SPI_Transmit(&hspi1, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(hspi1.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI1_CSB_GPIO_Port, SPI1_CSB_Pin, GPIO_PIN_SET); // NSS high

    return 0;
}

#endif
