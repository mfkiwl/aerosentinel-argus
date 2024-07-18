/*
 * bno055_main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS_H/BNO055/bno055_main.h"
#include <string.h>


struct bno055_t bno055_dev;

extern I2C_HandleTypeDef hi2c2;
#define BNO_I2C_HANDLE	(hi2c2)
uint8_t BNO_GTXBuffer[512], BNO_GRXBuffer[2048];

/* Variable used to return value of
* communication routine*/
s32 comres = BNO055_ERROR;

/* variable used to set the power mode of the sensor*/
u8 power_mode = BNO055_INIT_VALUE;

/*********read raw accel data***********/
/* variable used to read the accel x data */
s16 accel_datax = BNO055_INIT_VALUE;

/* variable used to read the accel y data */
s16 accel_datay = BNO055_INIT_VALUE;

/* variable used to read the accel z data */
s16 accel_dataz = BNO055_INIT_VALUE;

/* variable used to read the accel xyz data */
struct bno055_accel_t accel_xyz;

/*********read raw mag data***********/
/* variable used to read the mag x data */
s16 mag_datax = BNO055_INIT_VALUE;

/* variable used to read the mag y data */
s16 mag_datay = BNO055_INIT_VALUE;

/* variable used to read the mag z data */
s16 mag_dataz = BNO055_INIT_VALUE;

/* structure used to read the mag xyz data */
struct bno055_mag_t mag_xyz;

/***********read raw gyro data***********/
/* variable used to read the gyro x data */
s16 gyro_datax = BNO055_INIT_VALUE;

/* variable used to read the gyro y data */
s16 gyro_datay = BNO055_INIT_VALUE;

/* variable used to read the gyro z data */
s16 gyro_dataz = BNO055_INIT_VALUE;

/* structure used to read the gyro xyz data */
struct bno055_gyro_t gyro_xyz;

/*************read raw Euler data************/
/* variable used to read the euler h data */
s16 euler_data_h = BNO055_INIT_VALUE;

/* variable used to read the euler r data */
s16 euler_data_r = BNO055_INIT_VALUE;

/* variable used to read the euler p data */
s16 euler_data_p = BNO055_INIT_VALUE;

/* structure used to read the euler hrp data */
struct bno055_euler_t euler_hrp;

/************read raw quaternion data**************/
/* variable used to read the quaternion w data */
s16 quaternion_data_w = BNO055_INIT_VALUE;

/* variable used to read the quaternion x data */
s16 quaternion_data_x = BNO055_INIT_VALUE;

/* variable used to read the quaternion y data */
s16 quaternion_data_y = BNO055_INIT_VALUE;

/* variable used to read the quaternion z data */
s16 quaternion_data_z = BNO055_INIT_VALUE;

/* structure used to read the quaternion wxyz data */
struct bno055_quaternion_t quaternion_wxyz;

/************read raw linear acceleration data***********/
/* variable used to read the linear accel x data */
s16 linear_accel_data_x = BNO055_INIT_VALUE;

/* variable used to read the linear accel y data */
s16 linear_accel_data_y = BNO055_INIT_VALUE;

/* variable used to read the linear accel z data */
s16 linear_accel_data_z = BNO055_INIT_VALUE;

/* structure used to read the linear accel xyz data */
struct bno055_linear_accel_t linear_acce_xyz;

/*****************read raw gravity sensor data****************/
/* variable used to read the gravity x data */
s16 gravity_data_x = BNO055_INIT_VALUE;

/* variable used to read the gravity y data */
s16 gravity_data_y = BNO055_INIT_VALUE;

/* variable used to read the gravity z data */
s16 gravity_data_z = BNO055_INIT_VALUE;

/* structure used to read the gravity xyz data */
struct bno055_gravity_t gravity_xyz;

/*************read accel converted data***************/
/* variable used to read the accel x data output as m/s2 or mg */
double d_accel_datax = BNO055_INIT_VALUE;

/* variable used to read the accel y data output as m/s2 or mg */
double d_accel_datay = BNO055_INIT_VALUE;

/* variable used to read the accel z data output as m/s2 or mg */
double d_accel_dataz = BNO055_INIT_VALUE;

/* structure used to read the accel xyz data output as m/s2 or mg */
struct bno055_accel_double_t d_accel_xyz;

/******************read mag converted data********************/
/* variable used to read the mag x data output as uT*/
double d_mag_datax = BNO055_INIT_VALUE;

/* variable used to read the mag y data output as uT*/
double d_mag_datay = BNO055_INIT_VALUE;

/* variable used to read the mag z data output as uT*/
double d_mag_dataz = BNO055_INIT_VALUE;

/* structure used to read the mag xyz data output as uT*/
struct bno055_mag_double_t d_mag_xyz;

/*****************read gyro converted data************************/
/* variable used to read the gyro x data output as dps or rps */
double d_gyro_datax = BNO055_INIT_VALUE;

/* variable used to read the gyro y data output as dps or rps */
double d_gyro_datay = BNO055_INIT_VALUE;

/* variable used to read the gyro z data output as dps or rps */
double d_gyro_dataz = BNO055_INIT_VALUE;

/* structure used to read the gyro xyz data output as dps or rps */
struct bno055_gyro_double_t d_gyro_xyz;

/*******************read euler converted data*******************/

/* variable used to read the euler h data output
* as degree or radians*/
double d_euler_data_h = BNO055_INIT_VALUE;

/* variable used to read the euler r data output
* as degree or radians*/
double d_euler_data_r = BNO055_INIT_VALUE;

/* variable used to read the euler p data output
* as degree or radians*/
double d_euler_data_p = BNO055_INIT_VALUE;

/* structure used to read the euler hrp data output
* as as degree or radians */
struct bno055_euler_double_t d_euler_hpr;

/*********read linear acceleration converted data**********/
/* variable used to read the linear accel x data output as m/s2*/
double d_linear_accel_datax = BNO055_INIT_VALUE;

/* variable used to read the linear accel y data output as m/s2*/
double d_linear_accel_datay = BNO055_INIT_VALUE;

/* variable used to read the linear accel z data output as m/s2*/
double d_linear_accel_dataz = BNO055_INIT_VALUE;

/* structure used to read the linear accel xyz data output as m/s2*/
struct bno055_linear_accel_double_t d_linear_accel_xyz;

/********************Gravity converted data**********************/
/* variable used to read the gravity sensor x data output as m/s2*/
double d_gravity_data_x = BNO055_INIT_VALUE;

/* variable used to read the gravity sensor y data output as m/s2*/
double d_gravity_data_y = BNO055_INIT_VALUE;

/* variable used to read the gravity sensor z data output as m/s2*/
double d_gravity_data_z = BNO055_INIT_VALUE;

/* structure used to read the gravity xyz data output as m/s2*/
struct bno055_gravity_double_t d_gravity_xyz;


/**
  * @defgroup    ASM330LH I2C Platform Read/Write Group
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
signed char bno055_platform_read(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char ReadNumbr)
{
    uint16_t DevAddress = slave_address7 << 1;

    // send register address
    HAL_I2C_Master_Transmit(&BNO_I2C_HANDLE, DevAddress, &subaddress, 1, BUS_TIMEOUT);
    HAL_I2C_Master_Receive(&BNO_I2C_HANDLE, DevAddress, pBuffer, ReadNumbr, BUS_TIMEOUT);
    return 0;
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


signed char bno055_platform_write(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char WriteNumbr)
{
    uint16_t DevAddress = slave_address7 << 1;

    BNO_GTXBuffer[0] = subaddress;
    memcpy(&BNO_GTXBuffer[1], pBuffer, WriteNumbr);

    // send register address
    HAL_I2C_Master_Transmit(&BNO_I2C_HANDLE, DevAddress, BNO_GTXBuffer, WriteNumbr+1, BUS_TIMEOUT);
    return 0;
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


void DelayUs(unsigned int Delay)
{
	uint32_t i;

	while(Delay--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}


/**
  * @brief  BNO055 Initialization Function
  *
  * @param  		bno055_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 1 or True -> no Error)
  *
  */
int8_t BNO055_Init(struct bno055_t *dev){
	dev->bus_write = bno055_platform_write;
	dev->bus_read = bno055_platform_read;
	dev->delay_msec = DelayUs;
	dev->dev_addr = BNO055_I2C_ADDR1;

	uint8_t value;
	bno055_platform_read(BNO055_I2C_ADDR1, 0x00, &value, 1);

	comres = bno055_init(dev);
	if(comres == BNO055_SUCCESS)
	{
		printf("bno055_init success, chip id=%02X\r\n", dev->chip_id);
	}
	else
	{
		printf("bno055_init failed, comres=%d\r\n", comres);
	}

	return comres;
}
//
//int8_t bno055_interface_init(struct bno055_t *bmi, uint8_t intf)
//{
//	int8_t rslt = BMI3_OK;
//
//	/* Bus configuration : I2C */
//	if (intf == BMI3_I2C_INTF)
//	{
//		printf("I2C Interface \n");
//
//		/* To initialize the user I2C function */
//		bmi323_dev_addr = BMI3_ADDR_I2C_SEC;
//		bmi->intf = BMI3_I2C_INTF;
//		bmi->read = (bmi3_read_fptr_t)BMI323_SensorAPI_I2Cx_Read;
//		bmi->write = (bmi3_write_fptr_t)BMI323_SensorAPI_I2Cx_Write;
//	}
//	/* Bus configuration : SPI */
//	else if (intf == BMI3_SPI_INTF)
//	{
//		printf("SPI Interface \n");
//		bmi->intf = BMI3_SPI_INTF;
//		bmi->read = (bmi3_read_fptr_t)SensorAPI_SPIx_Read;
//		bmi->write = (bmi3_write_fptr_t)SensorAPI_SPIx_Write;
//	}
//
//
//	/* Assign device address to interface pointer */
//	bmi->intf_ptr = &bmi323_dev_addr;
//
//	/* Configure delay in microseconds */
//	bmi->delay_us = bst_delay_us;
//
//	/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
//	bmi->read_write_len = READ_WRITE_LEN;
//
//	return rslt;
//}


