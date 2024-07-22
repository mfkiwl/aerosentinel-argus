/*
 * bmi323_main.c
 *
 *  Created on: Jul 15, 2024
 *      Author: Yassine DEHHANI
 */



#include "DRIVERS_H/BMI323/bmi323_main.h"
#include "DRIVERS_H/BMI323/bmi323.h"
#include "DRIVERS_H/BMI323/common_porting.h"
#include "DRIVERS_H/BMI323/user_define.h"
#include "DRIVERS_H/BMI323/bmi3_defs.h"

//extern SPI_HandleTypeDef hspi1;
struct bmi3_dev bmi323;


struct bmi3_dev bmi323dev;
struct bmi3_dev *dev = &bmi323dev;
uint8_t bmi323_dev_addr;

struct bmi3_sensor_data sensor_data = { 0 };
struct bmi3_sensor_data sensor_data_AG[3] = { 0 };
float temperature_value;


/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

float accel_lsb_to_mps2_2g(int16_t val) {
    return (val / 16384.0f) * GRAVITY_EARTH;
}

float accel_lsb_to_mps2_4g(int16_t val) {
    return (val / 8192.0f) * GRAVITY_EARTH;
}

float accel_lsb_to_mps2_8g(int16_t val) {
    return (val / 4096.0f) * GRAVITY_EARTH;
}

float accel_lsb_to_mps2_16g(int16_t val) {
    return (val / 2048.0f) * GRAVITY_EARTH;
}
/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */


float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

float gyro_lsb_to_dps_125(int16_t val) {
    return val / 262.1f;
}

float gyro_lsb_to_dps_250(int16_t val) {
    return val / 131.1f;
}

float gyro_lsb_to_dps_500(int16_t val) {
    return val / 65.5f;
}

float gyro_lsb_to_dps_1000(int16_t val) {
    return val / 32.8f;
}

float gyro_lsb_to_dps_2000(int16_t val) {
    return val / 16.4f;
}

int8_t bmi3_interface_init(struct bmi3_dev *bmi, uint8_t intf)
{
	int8_t rslt = BMI3_OK;

		bmi->intf = BMI3_SPI_INTF;
		bmi->read = (bmi3_read_fptr_t)SensorAPI_SPIx_Read;
		bmi->write = (bmi3_write_fptr_t)SensorAPI_SPIx_Write;

	/* Assign device address to interface pointer */
	bmi->intf_ptr = &bmi323_dev_addr;

	/* Configure delay in microseconds */
	bmi->delay_us = bst_delay_us;

	/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
	bmi->read_write_len = READ_WRITE_LEN;

	return rslt;
}

int8_t Init_BMI323()
{ // START INIT
	int8_t rslt = BMI3_OK;
	uint8_t chipid;


	rslt = bmi3_interface_init(dev, BMI3_SPI_INTF);

	bst_delay_us(100000, dev->intf_ptr);

	/* Initialize bmi323. */
	rslt = bmi323_init(dev);

	if (rslt != BMI3_OK)
	{
		printf("bmi323_init() failed, error code: %d\r\n", rslt);
		return rslt;
	}

	rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, &chipid, 1, dev);
	if (rslt != BMI3_OK)
	{
		printf("read chip ID failed, error code: %d\r\n", rslt);
		return rslt;
	}

	//printf("Chip ID:0x%02x\r\n", chipid);

	//rslt = bmi323_get_config_version(&ver_major, &ver_minor, dev);
	//printf("The firmware version: v%d.%d\r\n", ver_major, ver_minor);

	/*Axis mapping according phisical structure*/
#if 0//For axis remap
	/* Strusture instance of axes remap */
	struct bmi3_axes_remap remap = { 0 };

	rslt = bmi323_get_remap_axes(&remap, &dev);
	//printf("get_remap_axes: %d \n", rslt);

	/* @note: XYZ axis denotes x = x, y = y, z = z
	* Similarly,
	*    ZYX, x = z, y = y, z = x
	*/
	remap.axis_map = BMI3_MAP_ZYX_AXIS;

	/* Invert the x axis of accelerometer and gyroscope */
	remap.invert_x = BMI3_MAP_NEGATIVE;

	/* Invert the y axis of accelerometer and gyroscope */
	remap.invert_y = BMI3_MAP_NEGATIVE;

	/* Invert the z axis of accelerometer and gyroscope */
	remap.invert_z = BMI3_MAP_NEGATIVE;

	if (rslt == BMI323_OK)
	{
		rslt = bmi323_set_remap_axes(remap, &dev);
		//printf("set_remap_axes: %d \n", rslt);

		if (rslt != BMI323_OK)
		{
			//printf("bmi323_set_remap_axes() failed\r\n");
		}
	}
	else
	{
		//printf("bmi323_get_remap_axes() failed\r\n");
	}

#endif
#if defined(ACC_GYRO_SELFTEST)
	rslt = bmi323_perform_self_test(BMI3_ST_BOTH_ACC_GYR, &st_result_status, dev);
	//printf("Perform_self_test: %d \n", rslt);

	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
	{
		//printf("ACC self-test is successfully completed \n");
	}
	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
	{
		//printf("Self-test is not successfully completed\n");

		switch (st_result_status.self_test_err_rslt)
		{
			case BMI3_SC_ST_ABORTED_MASK:
				//printf("SC_ST_ABORTED\n");
				break;
			case BMI3_ST_IGNORED_MASK:
				//printf("BMI323_ST_IGNORED\n");
				break;
			case BMI3_SC_ST_PRECON_ERR_MASK:
				//printf("BMI323_SC_ST_PRECON_ERR\n");
				break;
			case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
				//printf("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
				break;
			default:
				break;
		}
	}

	//printf("Result of acc_x_axis is %d\n", st_result_status.acc_sens_x_ok);
	//printf("Result of acc_y_axis is %d\n", st_result_status.acc_sens_y_ok);
	//printf("Result of acc_z_axis is %d\n", st_result_status.acc_sens_z_ok);
	//printf("Result of gyr_x_axis is %d\n", st_result_status.gyr_sens_x_ok);
	//printf("Result of gyr_y_axis is %d\n", st_result_status.gyr_sens_y_ok);
	//printf("Result of gyr_z_axis is %d\n", st_result_status.gyr_sens_z_ok);
	//printf("Result of gyr_drive_ok is %d\n", st_result_status.gyr_drive_ok);
	//printf("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
	//printf("Result of ST_result is %d\n", st_result_status.self_test_rslt);
#endif

	#if defined(ACC_GYRO)
	Open_BMI323_ACC(dev);
	Open_BMI323_GYRO(dev);
	#endif

	return rslt;
}

int8_t Open_BMI323_ACC()
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_ACCEL;

	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK)
	{
		/* Update all or any of the accelerometer
		configurations */
	#if defined(SUPPORT_LOWPOWER)
		/* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
		config.cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

		/* Set number of average samples for accel. */
		config.cfg.acc.avg_num = BMI3_ACC_AVG1;

		/* Enable the accel mode where averaging of samples
		* will be done based on above set bandwidth and ODR.
		* Note : By default accel is disabled. The accel will get enable by selecting the mode.
		*/
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_LOW_PWR;
	#else
		/* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
		config.cfg.acc.bwp = BMI3_ACC_BW_ODR_HALF;

		/* Set number of average samples for accel. */
		config.cfg.acc.avg_num = BMI3_ACC_AVG4;

		/* Enable the accel mode where averaging of samples
		* will be done based on above set bandwidth and ODR.
		* Note : By default accel is disabled. The accel will get enable by selecting the mode.
		*/
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_HIGH_PERF;
	#endif


		config.cfg.acc.odr      = BMI3_ACC_ODR_50HZ;

		/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
		config.cfg.acc.range     = BMI3_ACC_RANGE_4G;

		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK)
		{
			//printf("Open ACC failed, rslt=%d\r\n", rslt);
		}
		else
		{
			//printf("Open ACC set successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK)
			{
				//printf("Get ACC configuration successful\r\n");
				//printf("acc_mode = %d\r\n", config.cfg.acc.acc_mode);
				//printf("bwp = %d\r\n", config.cfg.acc.bwp);
				//printf("odr = %d\r\n", config.cfg.acc.odr);
				//printf("Range = %d\r\n", config.cfg.acc.range);
				//printf("avg_num = %d\r\n", config.cfg.acc.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_ACC()
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_ACCEL;
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK)
	{
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_DISABLE;

		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK)
		{
			//printf("Close ACC failed, rslt=%d\r\n", rslt);
		}
		else
		{
			//printf("Open ACC successfully\r\n");
		}
	}
	//printf("Close_BMI323_ACC: %d \n",rslt);

	return rslt;
}

int8_t Open_BMI323_GYRO()
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_GYRO;

	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK)
	{
		config.cfg.gyr.odr = BMI3_GYR_ODR_50HZ;
		/* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
		config.cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;

	#if defined(SUPPORT_LOWPOWER)
		/*	The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
		*	Value	Name	  Description
		*	  0   odr_half	 BW = gyr_odr/2
		*	  1  odr_quarter BW = gyr_odr/4
		*/
		config.cfg.gyr.bwp = BMI3_GYR_BW_ODR_QUARTER;
		/* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_LOW_PWR;
		/* Value    Name    Description
		*  000     avg_1   No averaging; pass sample without filtering
		*  001     avg_2   Averaging of 2 samples
		*  010     avg_4   Averaging of 4 samples
		*  011     avg_8   Averaging of 8 samples
		*  100     avg_16  Averaging of 16 samples
		*  101     avg_32  Averaging of 32 samples
		*  110     avg_64  Averaging of 64 samples
		*/
		config.cfg.gyr.avg_num = BMI3_GYR_AVG1;
	#else
		/*	The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
		*	Value	Name	  Description
		*	  0   odr_half	 BW = gyr_odr/2
		*	  1  odr_quarter BW = gyr_odr/4
		*/
		config.cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;
		/* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_HIGH_PERF;
		/* Value    Name    Description
		*  000     avg_1   No averaging; pass sample without filtering
		*  001     avg_2   Averaging of 2 samples
		*  010     avg_4   Averaging of 4 samples
		*  011     avg_8   Averaging of 8 samples
		*  100     avg_16  Averaging of 16 samples
		*  101     avg_32  Averaging of 32 samples
		*  110     avg_64  Averaging of 64 samples
		*/
		config.cfg.gyr.avg_num = BMI3_GYR_AVG4;
	#endif

		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK)
		{
			//printf("Open GYRO failed\r\n");
		}
		else
		{
			//printf("Open GYRO successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK)
			{
				//printf("Get BMI2_GYRO Configuration successful\r\n");
				//printf("gyr_mode = %d\r\n", config.cfg.gyr.gyr_mode);
				//printf("ODR = %d\r\n", config.cfg.gyr.odr);
				//printf("Range = %d\r\n", config.cfg.gyr.range);
				//printf("bwp = %d\r\n", config.cfg.gyr.bwp);
				//printf("avg_num = %d\r\n", config.cfg.gyr.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_GYRO()
{
	int8_t rslt = BMI3_OK;

	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_GYRO;
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK)
	{
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_DISABLE;

		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK)
		{
			//printf("Close GYRO failed, rslt=%d\r\n", rslt);
		}
		else
		{
			//printf("Open GYRO successfully\r\n");
		}
	}
	//printf("Close_BMI323_GYRO: %d \n",rslt);

	return rslt;
}


// Function to get and return the sensor data
IMU_6_Axis_Data bmi323_data_poll() {
    IMU_6_Axis_Data imu_data = {0};
    struct bmi3_sensor_data sensor_data[2];

    // Set the types of data we want to read
    sensor_data[0].type = BMI3_ACCEL;
    sensor_data[1].type = BMI3_GYRO;

    // Get the sensor data
    int8_t rslt = bmi3_get_sensor_data(sensor_data, 2, dev);

    if (rslt == BMI3_OK) {
        // Use the appropriate conversion functions
        // Replace these with the actual functions based on your configuration

        // Fill accelerometer data
        if (sensor_data[0].type == BMI3_ACCEL) {
            imu_data.acceleration[0] = accel_lsb_to_mps2_4g(sensor_data[0].sens_data.acc.x);  // Assuming ±4g range
            imu_data.acceleration[1] = accel_lsb_to_mps2_4g(sensor_data[0].sens_data.acc.y);  // Assuming ±4g range
            imu_data.acceleration[2] = accel_lsb_to_mps2_4g(sensor_data[0].sens_data.acc.z);  // Assuming ±4g range
        }

        // Fill gyroscope data
        if (sensor_data[1].type == BMI3_GYRO) {
            imu_data.gyroscope[0] = gyro_lsb_to_dps_2000(sensor_data[1].sens_data.gyr.x);  // Assuming ±2000°/s range
            imu_data.gyroscope[1] = gyro_lsb_to_dps_2000(sensor_data[1].sens_data.gyr.y);  // Assuming ±2000°/s range
            imu_data.gyroscope[2] = gyro_lsb_to_dps_2000(sensor_data[1].sens_data.gyr.z);  // Assuming ±2000°/s range
        }
    } else {
        //printf("Failed to get sensor data. Error code: %d\n", rslt);
    }

    return imu_data;
}


void bmi323_print_sensor_data(IMU_6_Axis_Data *data){
	printf("BMI323 IMU: \n");
    printf("Acceleration: -> ");
    printf("X: %f m/s^2, Y: %f m/s^2, Z: %f m/s^2\n", data->acceleration[0], data->acceleration[1], data->acceleration[2]);
    printf("Angular Rate -> ");
    printf("X: %f dps, Y: %f dps, Z: %f dps\n", data->gyroscope[0], data->gyroscope[1], data->gyroscope[2]);
    printf("----- \n");
}
