/*
 * bno055_main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS/BNO055/bno055_main.h"
#include <string.h>


struct bno055_t bno055_dev;

extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c4;
#define BNO_I2C_HANDLE	(hi2c4)

uint8_t BNO_GTXBuffer[512], BNO_GRXBuffer[2048];
struct bno055_euler_float_t *euler_temp_data;
struct bno055_quaternion_t *quaternion_temp_data;
struct bno055_accel_double_t *accel_temp_data;
struct bno055_gyro_double_t *gyro_temp_xyz;
struct bno055_gravity_float_t *gravity_temp_data;
double *mag_x_temp_data;
double *mag_y_temp_data;
double *mag_z_temp_data;
float *temp_temp_data;

/* Variable used to return value of communication routine*/
s32 comres = BNO055_ERROR;

/* Variable used to return value of current operational mode*/
u8 op_mode_current = 0;

/* variable used to set the power mode of the sensor*/
u8 power_mode = BNO055_INIT_VALUE;

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
//	uint32_t i;
//
//	while(Delay--)
//	{
//		for(i = 0; i < 84; i++)
//		{
//			;
//		}
//	}

	HAL_Delay(Delay);
}


/**
  * @brief  BNO055 Initialization Function
  *
  * @param  		bno055_config     TODO -> Create this structure and add the parameter to this function.
  * @retval         Initialization Status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t BNO055_Init(){
	bno055_dev.bus_write = bno055_platform_write;
	bno055_dev.bus_read = bno055_platform_read;
	bno055_dev.delay_msec = &DelayUs;
	bno055_dev.dev_addr = BNO055_I2C_ADDR1;

	uint8_t value;
	bno055_platform_read(BNO055_I2C_ADDR1, 0x00, &value, 1);

	comres = bno055_init(&bno055_dev);
//	if(comres == BNO055_SUCCESS)
//	{
//		printf("bno055_init success, chip id=%02X\r\n", bno055_dev.chip_id);
//	}
//	else
//	{
//		printf("bno055_init failed, comres=%d\r\n", comres);
//	}

	/* set the power mode as NORMAL*/
	power_mode = BNO055_POWER_MODE_NORMAL;
	comres += bno055_set_power_mode(power_mode);
//	if(comres == BNO055_SUCCESS)
//	{
//		printf("BNO Power mode set successfully! \n");
//	}
//	else
//	{
//		printf("bno055_init failed, comres=%d\r\n", comres);
//	}

	/************************* START READ RAW FUSION DATA ********
		* For reading fusion data it is required to set the
		* operation modes of the sensor
		* operation mode can set from the register
		* page - page0
		* register - 0x3D
		* bit - 0 to 3
		* for sensor data read following operation mode have to set
		* FUSION MODE
		* 0x08 - BNO055_OPERATION_MODE_IMUPLUS
		* 0x09 - BNO055_OPERATION_MODE_COMPASS
		* 0x0A - BNO055_OPERATION_MODE_M4G
		* 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
		* 0x0C - BNO055_OPERATION_MODE_NDOF
		* based on the user need configure the operation mode*/
	    // Set the operation mode to NDOF
		printf("Trying to set BNO055 operational mode...");
		while(op_mode_current != BNO055_OPERATION_MODE_NDOF){
		    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
		    comres += bno055_get_operation_mode(&op_mode_current);
		    HAL_Delay(250);
		}

		printf("Success! \n");

	    HAL_Delay(1000);

//		comres +=bno055_set_temp_unit(BNO055_TEMP_UNIT_CELSIUS);
//		comres +=bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
//		comres +=bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
//		comres +=bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
//		if(comres == BNO055_SUCCESS)
//		{
//			printf("BNO Units configured successfully \n");
//		}
//		else
//		{
//			printf("BNO Units configuration failed\r\n", comres);
//		}



	return (int8_t)comres;
}

AHRS_9_Axis_Data bno_read_fusion_data(){

    int8_t comres = 0;

    // Initialize the return structure
    AHRS_9_Axis_Data data = {0};

    // Read Euler angles (Orientation)
    comres += bno055_convert_float_euler_hpr_deg(euler_temp_data);
    data.orientation[0] = euler_temp_data->p;
    data.orientation[1] = euler_temp_data->r;
    data.orientation[2] = euler_temp_data->h;

    // Read Quaternion data -> Added scaling factor division for quaternion normalization (-1 ; +1)
    comres += bno055_read_quaternion_wxyz(quaternion_temp_data);
    data.quaternion.w = quaternion_temp_data->w / QUAT_SCALING_FACT;
    data.quaternion.x = quaternion_temp_data->x / QUAT_SCALING_FACT;
    data.quaternion.y = quaternion_temp_data->y / QUAT_SCALING_FACT;
    data.quaternion.z = quaternion_temp_data->z / QUAT_SCALING_FACT;

    // Read Linear acceleration
    comres += bno055_convert_double_accel_xyz_mg(accel_temp_data);
    data.acceleration[0] = accel_temp_data->x / 1000; // Division by 1000 -> Converts millig to g
    data.acceleration[1] = accel_temp_data->y / 1000; // Division by 1000 -> Converts millig to g
    data.acceleration[2] = accel_temp_data->z / 1000; // Division by 1000 -> Converts millig to g

    comres += bno055_convert_double_gyro_xyz_dps(gyro_temp_xyz);
    data.gyroscope[0] = gyro_temp_xyz->x;
    data.gyroscope[1] = gyro_temp_xyz->y;
    data.gyroscope[2] = gyro_temp_xyz->z;

    // Read Gravity data
    comres += bno055_convert_float_gravity_xyz_msq(gravity_temp_data);
    data.gravity[0] = gravity_temp_data->x;
    data.gravity[1] = gravity_temp_data->y;
    data.gravity[2] = gravity_temp_data->z;

    // Read Magnetometer data
    comres += bno055_convert_double_mag_x_uT(&data.magnetometer[0]);
    comres += bno055_convert_double_mag_y_uT(&data.magnetometer[1]);
    comres += bno055_convert_double_mag_z_uT(&data.magnetometer[2]);

    comres += bno055_convert_float_temp_celsius(&data.temperature);

    // Check communication results (Optional, for debugging)
    if (comres != 0) {
        //printf("Communication error: %d\n", comres);
    }

    return data;
}

//void Send_Quat_To_IMU_Visualizer(quaternion_vect q)
//{
//char quaternionStr[100];
//float lama = 0.1;
//// Format quaternion (W, X, Y, Z vectors) as a comma-separated string
//sprintf(quaternionStr, "%.6f,%.6f,%.6f,%.6f,%.6f\r\n", q.w, q.x, q.y, q.z, lama);
//HAL_UART_Transmit(&huart1, (uint8_t*)quaternionStr, strlen(quaternionStr), HAL_MAX_DELAY);
//}

void bno055_print_fusion_data(AHRS_9_Axis_Data *data) {
printf("BNO055 AHRS: \n");
// Print orientation (Pitch, Roll, Yaw)
printf("Orientation -> ");
printf("Pitch: %.2f deg, Roll: %.2f deg, Yaw: %.2f deg \n", data->orientation[0], data->orientation[1], data->orientation[2]);

//printf("Orientation from Quaternions -> ");
//printf("Pitch: %.2f deg, Roll: %.2f deg, Yaw: %.2f deg \n", data->orientation_q[0], data->orientation_q[1], data->orientation_q[2]);

// Print quaternion (W, X, Y, Z vectors)
printf("Quaternion -> ");
printf("W: %.2f , X: %.2f , Y: %.2f , Z: %.2f \n", data->quaternion.w, data->quaternion.x, data->quaternion.y, data->quaternion.z);

//Send_Quat_To_IMU_Visualizer(data->quaternion);

// Print acceleration (X, Y, Z)
printf("Acceleration -> ");
printf("X: %.2f g, Y: %.2f g, Z: %.2f g \n", data->acceleration[0], data->acceleration[1], data->acceleration[2]);

// Print gyroscope data (X, Y, Z)
printf("Angular Rate -> ");
printf("X: %.2f dps, Y: %.2f dps, Z: %.2f dps \n", data->gyroscope[0], data->gyroscope[1], data->gyroscope[2]);

// Print gravity data (X, Y, Z)
printf("Gravity -> ");
printf("X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2\n", data->gravity[0], data->gravity[1], data->gravity[2]);

// Print magnetometer data (X, Y, Z)
printf("Magnetometer -> ");
printf("X: %.2f µT, Y: %.2f µT, Z: %.2f µT \n", data->magnetometer[0], data->magnetometer[1], data->magnetometer[2]);

// Print magnetometer data (X, Y, Z)
printf("Temperature -> ");
printf("Temperature: %.2f degC \n", data->temperature);

printf("----- \n");
}
