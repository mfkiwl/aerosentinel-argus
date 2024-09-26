/*
 * ADXL375.c
 *
 *  Created on: Apr 8, 2024
 *      Author: Yassine DEHHANI
 */


#include "DRIVERS/ADXL375/ADXL375.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
HAL_StatusTypeDef status;

ADXL375 dev;			/* structure instance for data storage */

int errors;		/* Initialisation error check */


/*
 * INITIALISATION
 */

int8_t ADXL375_Init(){
    ToggleCSHigh();

    /* Use the global variable 'hspi1' directly */
    dev.spiHandle = &hspi1;

    HAL_StatusTypeDef status;

    int8_t errorNum = 0;

    /* confirm device address as 0xE5 */
    uint8_t confirmID;
    while(1) {
        status = ReadData(ADXL375_DEVID, &confirmID, 1);
        if (status == HAL_OK) {
            if (confirmID == 0xE5)
                break;
        }
    }

    /* set the required data rate */
    status = WriteData(ADXL375_BW_RATE, BW_RATE_DATA_RATE, 1);
    if (status != HAL_OK) {
        errorNum++;
    }

    /* if all well till now, enable measurement mode */
    status = WriteData(ADXL375_POWER_CTL, POWER_CTL_MEASURE_MODE, 1);
    if (status != HAL_OK)
        errorNum++;

    return errorNum;
}



/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef ADXL375_ReadAcceleration()
{
    /* Use the global 'dev' instance */
    HAL_StatusTypeDef status = ReadData(ADXL375_DATAX0, dev.rawAccData, 6);
    return status;
}


void ADXL375_CleanRawValues()
{
    int16_t val;

    /* CONVERSION FOR ACC_X */
    val = dev.rawAccData[1];
    val = (val << 8) + (dev.rawAccData[0]);
    if (val & 0x8000) {
        val = -((~val & 0xFFFF) + 1);
    }
    dev.accData[0] = (float)val * 49.0f;

    /* CONVERSION FOR ACC_Y */
    val = dev.rawAccData[3];
    val = (val << 8) + (dev.rawAccData[2]);
    if (val & 0x8000) {
        val = -((~val & 0xFFFF) + 1);
    }
    dev.accData[1] = (float)val * 49.0f;

    /* CONVERSION FOR ACC_Z */
    val = dev.rawAccData[5];
    val = (val << 8) + (dev.rawAccData[4]);
    if (val & 0x8000) {
        val = -((~val & 0xFFFF) + 1);
    }
    dev.accData[2] = (float)val * 49.0f;
}


HigG_Accemerometer_3_Axis get_high_g_acceleration(){
	HigG_Accemerometer_3_Axis data;

	  status = ADXL375_ReadAcceleration();
	  if (status == HAL_OK) {
		  ADXL375_CleanRawValues();
		  data.acc.x = dev.accData[0];
		  data.acc.y = dev.accData[1];
		  data.acc.z = dev.accData[2];
	  }

	  return data;
}




void adxl375_print_highG_data(HigG_Accemerometer_3_Axis *data) {
printf("ADXL375 HIGH-G Accelerometer: \n");

// Print acceleration (X, Y, Z)
printf("Acceleration -> ");
printf("X: %.2f g, Y: %.2f g, Z: %.2f g \n", data->acc.x,data->acc.y,data->acc.z);
}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef WriteData(uint8_t address, uint8_t data, uint16_t len)
{
    ToggleCSLow();

    /* Use the global 'dev' instance */
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev.spiHandle, &address, 1, HAL_MAX_DELAY);
    if (status == HAL_OK)
        status = HAL_SPI_Transmit(dev.spiHandle, &data, 1, HAL_MAX_DELAY);

    ToggleCSHigh();

    return status;
}


HAL_StatusTypeDef ReadData(uint8_t address, uint8_t *data, uint16_t len)
{
    ToggleCSLow();

    uint8_t txBuffer = (address | 0x80);

    if (len > 1) {
        txBuffer = (txBuffer | 0xC0);
    }

    /* Use the global 'dev' instance */
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev.spiHandle, &txBuffer, 1, HAL_MAX_DELAY);
    HAL_Delay(3);
    if (status == HAL_OK)
        status = HAL_SPI_Receive(dev.spiHandle, data, len, HAL_MAX_DELAY);

    ToggleCSHigh();

    return status;
}


void ToggleCSHigh()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

void ToggleCSLow()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

