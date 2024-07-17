///*
// * bme680_main.c
// *
// *  Created on: Jul 14, 2024
// *      Author: Yassine DEHHANI
// */
//
//#include "DRIVERS_H/BME680/bme680_main.h"
//#include "DRIVERS_H/BME680/bme680.h"
//#include "DRIVERS_H/BME680/bme680_defs.h"
//#include "main.h" // Include the header that defines SPI2_CSB_Pin and SPI2_CSB_GPIO_Port
//
//extern SPI_HandleTypeDef hspi2;
//struct bme680_dev bme680;
////uint8_t GTXBuffer[512], GRXBuffer[512];
//
//void CSB_SetHigh(void) {
//    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_SET);
//}
//
//void CSB_SetLow(void) {
//    HAL_GPIO_WritePin(SPI2_CSB_GPIO_Port, SPI2_CSB_Pin, GPIO_PIN_RESET);
//}
//
///**
//  * @defgroup    BME680 SPI Platform Read/Write Group
//  * @brief       This section provide a set of functions used to read on the ASM330LHH I2C Port
//  *              MANDATORY: return 0 -> no Error.
//  * @{
//  *
//  */
//
//int8_t bme680_platform_read(uint8_t slave_address7, uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr)
//{
//GTXBuffer[0] = subaddress | 0x80;
//
//CSB_SetLow(); // NSS low
//
//HAL_SPI_TransmitReceive(&hspi2, GTXBuffer, GRXBuffer, ReadNumbr+1, HAL_MAX_DELAY); // timeout 1000msec;
//while(hspi2.State == HAL_SPI_STATE_BUSY); // wait for xmission complete
//
//CSB_SetHigh(); // NSS high
//memcpy(pBuffer, GRXBuffer+1, ReadNumbr);
//
//return 0;
//}
//
//int8_t bme680_platform_write(uint8_t slave_address7, uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr)
//{
//GTXBuffer[0] = subaddress & 0x7F;
//memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);
//
//CSB_SetLow(); // NSS low
//
////HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
//HAL_SPI_Transmit(&hspi2, GTXBuffer, WriteNumbr+1, HAL_MAX_DELAY); // send register address + write data
//while(hspi2.State == HAL_SPI_STATE_BUSY); // wait for xmission complete
//
//CSB_SetHigh(); // NSS high
//
//return 0;
//}
///**
//  * @}
//  *
//  */
//
//void bme680_delay_func(uint32_t period) {
//    HAL_Delay(period); // Assuming HAL_Delay is in milliseconds
//}
//
//
///**
//  * @brief  BME680 Initialization Function
//  *
//  * @param  bme680_config     TODO -> Create this structure and add the parameter to this function.
//  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
//  *
//  */
//bool BME680_Init(){
//
//	bme680.dev_id = 0;
//    bme680.intf = BME680_SPI_INTF;
//    bme680.delay_ms = bme680_delay_func;
//
//	//Assign Read/Write functions to the device
//    bme680.read = bme680_platform_read;
//    bme680.write = bme680_platform_write;
//
//    int bme680_status = bme680_init(&bme680);
//    printf("BME680 STATUS = %d",bme680_status);
//
//
//    return !bme680_status;
//}
