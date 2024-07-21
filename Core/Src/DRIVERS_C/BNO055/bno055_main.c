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

/* Variable used to return value of communication routine*/
s32 comres = BNO055_ERROR;
/* Variable used to return value of current operational mode*/
u8 op_mode_current = 0;

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
int8_t BNO055_Init(){
	bno055_dev.bus_write = bno055_platform_write;
	bno055_dev.bus_read = bno055_platform_read;
	bno055_dev.delay_msec = &DelayUs;
	bno055_dev.dev_addr = BNO055_I2C_ADDR1;

	uint8_t value;
	bno055_platform_read(BNO055_I2C_ADDR1, 0x00, &value, 1);

	comres = bno055_init(&bno055_dev);
	if(comres == BNO055_SUCCESS)
	{
		printf("bno055_init success, chip id=%02X\r\n", bno055_dev.chip_id);
	}
	else
	{
		printf("bno055_init failed, comres=%d\r\n", comres);
	}

	/* set the power mode as NORMAL*/
	power_mode = BNO055_POWER_MODE_NORMAL;
	comres += bno055_set_power_mode(power_mode);
	if(comres == BNO055_SUCCESS)
	{
		//printf("BNO Power mode set successfully! \n");
	}
	else
	{
		//printf("bno055_init failed, comres=%d\r\n", comres);
	}

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
	    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	    comres += bno055_get_operation_mode(&op_mode_current);
		if(comres == BNO055_SUCCESS && op_mode_current == BNO055_OPERATION_MODE_NDOF)
		{
			printf("BNO Operation mode set successfully (NDOF Mode)! \n");
			printf("BNO Current Mode (Verif) : %d \n", op_mode_current);
		}
		else
		{
			printf("bno055_init failed, comres=%d\r\n", comres);
		}
		HAL_Delay(1000); // 1 second

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
    comres += bno055_read_euler_h(&euler_data_h);
    comres += bno055_read_euler_r(&euler_data_r);
    comres += bno055_read_euler_p(&euler_data_p);
    data.orientation[0] = euler_data_p;
    data.orientation[1] = euler_data_r;
    data.orientation[2] = euler_data_h;

    // Read Quaternion data (Optional, not used in this example)
    comres += bno055_read_quaternion_w(&quaternion_data_w);
    comres += bno055_read_quaternion_x(&quaternion_data_x);
    comres += bno055_read_quaternion_y(&quaternion_data_y);
    comres += bno055_read_quaternion_z(&quaternion_data_z);

    // Read Linear acceleration
    comres += bno055_read_linear_accel_x(&linear_accel_data_x);
    comres += bno055_read_linear_accel_y(&linear_accel_data_y);
    comres += bno055_read_linear_accel_z(&linear_accel_data_z);
    data.acceleration[0] = linear_accel_data_x;
    data.acceleration[1] = linear_accel_data_y;
    data.acceleration[2] = linear_accel_data_z;

    // Read Gravity data
    comres += bno055_read_gravity_x(&gravity_data_x);
    comres += bno055_read_gravity_y(&gravity_data_y);
    comres += bno055_read_gravity_z(&gravity_data_z);
    // Gravity data can be used as gyroscope data
    data.gyroscope[0] = gravity_data_x;
    data.gyroscope[1] = gravity_data_y;
    data.gyroscope[2] = gravity_data_z;

    // Read Magnetometer data
    comres += bno055_read_mag_x(&mag_datax);
    comres += bno055_read_mag_y(&mag_datay);
    comres += bno055_read_mag_z(&mag_dataz);
    data.magnetometer[0] = mag_datax;
    data.magnetometer[1] = mag_datay;
    data.magnetometer[2] = mag_dataz;

    // Check communication results (Optional, for debugging)
    if (comres != 0) {
        //printf("Communication error: %d\n", comres);
    }

    return data;
}

void bno055_print_fusion_data(AHRS_9_Axis_Data *data) {
printf("BNO055 AHRS: \n");
// Print orientation (Pitch, Roll, Yaw)
printf("Orientation -> ");
printf("Pitch: %.2f m/s^2, Roll: %.2f m/s^2, Yaw: %.2f m/s^2\n", data->orientation[0], data->orientation[1], data->orientation[2]);

// Print acceleration (X, Y, Z)
printf("Acceleration -> ");
printf("X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2\n", data->acceleration[0], data->acceleration[1], data->acceleration[2]);

// Print gyroscope data (X, Y, Z)
printf("Angular Rate -> ");
printf("X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2\n", data->gyroscope[0], data->gyroscope[1], data->gyroscope[2]);

// Print magnetometer data (X, Y, Z)
printf("Magnetometer -> ");
printf("X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2\n", data->magnetometer[0], data->magnetometer[1], data->magnetometer[2]);

printf("----- \n");
}


AHRS_Converted_Data bno_read_converted_data() {
    int8_t comres = 0;

    // Initialize the return structure
    AHRS_Converted_Data data = {0};

    // Read converted accelerometer data
    comres += bno055_convert_double_accel_x_mg(&data.accel_x_mg);
    comres += bno055_convert_double_accel_y_mg(&data.accel_y_mg);
    comres += bno055_convert_double_accel_z_mg(&data.accel_z_mg);
    comres += bno055_convert_double_accel_xyz_mg(&data.accel_xyz_mg);

    // Read converted magnetometer data
    comres += bno055_convert_double_mag_x_uT(&data.mag_x_uT);
    comres += bno055_convert_double_mag_y_uT(&data.mag_y_uT);
    comres += bno055_convert_double_mag_z_uT(&data.mag_z_uT);
    comres += bno055_convert_double_mag_xyz_uT(&data.mag_xyz_uT);

    // Read converted gyroscope data
    comres += bno055_convert_double_gyro_x_dps(&data.gyro_x_dps);
    comres += bno055_convert_double_gyro_y_dps(&data.gyro_y_dps);
    comres += bno055_convert_double_gyro_z_dps(&data.gyro_z_dps);
    comres += bno055_convert_double_gyro_xyz_dps(&data.gyro_xyz_dps);

    // Read converted Euler angles data
    comres += bno055_convert_double_euler_h_deg(&data.euler_h_deg);
    comres += bno055_convert_double_euler_r_deg(&data.euler_r_deg);
    comres += bno055_convert_double_euler_p_deg(&data.euler_p_deg);
    comres += bno055_convert_double_euler_hpr_deg(&data.euler_hpr_deg);

    // Read converted linear acceleration data
    comres += bno055_convert_double_linear_accel_x_msq(&data.linear_accel_x_ms2);
    comres += bno055_convert_double_linear_accel_y_msq(&data.linear_accel_y_ms2);
    comres += bno055_convert_double_linear_accel_z_msq(&data.linear_accel_z_ms2);
    comres += bno055_convert_double_linear_accel_xyz_msq(&data.linear_accel_xyz_ms2);

    // Read converted gravity data
    comres += bno055_convert_gravity_double_x_msq(&data.gravity_x_ms2);
    comres += bno055_convert_gravity_double_y_msq(&data.gravity_y_ms2);
    comres += bno055_convert_gravity_double_z_msq(&data.gravity_z_ms2);
    comres += bno055_convert_double_gravity_xyz_msq(&data.gravity_xyz_ms2);

    // Check communication results (Optional, for debugging)
    if (comres != BNO055_SUCCESS) {
        printf("Communication error: %d\n", comres);
    }

    return data;
}


void print_bno055_data(AHRS_Converted_Data *data) {
    // Print accelerometer data
    printf("Accelerometer Data:\n");
    printf("X: %.2f m/s^2, %.2f mg\n", data->accel_x_ms2, data->accel_x_mg);
    printf("Y: %.2f m/s^2, %.2f mg\n", data->accel_y_ms2, data->accel_y_mg);
    printf("Z: %.2f m/s^2, %.2f mg\n", data->accel_z_ms2, data->accel_z_mg);
    printf("XYZ: [%.2f, %.2f, %.2f] m/s^2\n", data->accel_xyz_ms2.x, data->accel_xyz_ms2.y, data->accel_xyz_ms2.z);
    printf("XYZ: [%.2f, %.2f, %.2f] mg\n", data->accel_xyz_mg.x, data->accel_xyz_mg.y, data->accel_xyz_mg.z);

    // Print magnetometer data
    printf("\nMagnetometer Data:\n");
    printf("X: %.2f uT\n", data->mag_x_uT);
    printf("Y: %.2f uT\n", data->mag_y_uT);
    printf("Z: %.2f uT\n", data->mag_z_uT);
    printf("XYZ: [%.2f, %.2f, %.2f] uT\n", data->mag_xyz_uT.x, data->mag_xyz_uT.y, data->mag_xyz_uT.z);

    // Print gyroscope data
    printf("\nGyroscope Data:\n");
    printf("X: %.2f dps, %.2f rps\n", data->gyro_x_dps, data->gyro_x_rps);
    printf("Y: %.2f dps, %.2f rps\n", data->gyro_y_dps, data->gyro_y_rps);
    printf("Z: %.2f dps, %.2f rps\n", data->gyro_z_dps, data->gyro_z_rps);
    printf("XYZ: [%.2f, %.2f, %.2f] dps\n", data->gyro_xyz_dps.x, data->gyro_xyz_dps.y, data->gyro_xyz_dps.z);
    printf("XYZ: [%.2f, %.2f, %.2f] rps\n", data->gyro_xyz_rps.x, data->gyro_xyz_rps.y, data->gyro_xyz_rps.z);

    // Print Euler angles data
    printf("\nEuler Angles Data:\n");
    printf("Heading: %.2f degrees, %.2f radians\n", data->euler_h_deg, data->euler_h_rad);
    printf("Roll: %.2f degrees, %.2f radians\n", data->euler_r_deg, data->euler_r_rad);
    printf("Pitch: %.2f degrees, %.2f radians\n", data->euler_p_deg, data->euler_p_rad);
    printf("HPR: [%.2f, %.2f, %.2f] degrees\n", data->euler_hpr_deg.h, data->euler_hpr_deg.r, data->euler_hpr_deg.p);
    printf("HPR: [%.2f, %.2f, %.2f] radians\n", data->euler_hpr_rad.h, data->euler_hpr_rad.r, data->euler_hpr_rad.p);

    // Print linear acceleration data
    printf("\nLinear Acceleration Data:\n");
    printf("X: %.2f m/s^2\n", data->linear_accel_x_ms2);
    printf("Y: %.2f m/s^2\n", data->linear_accel_y_ms2);
    printf("Z: %.2f m/s^2\n", data->linear_accel_z_ms2);
    printf("XYZ: [%.2f, %.2f, %.2f] m/s^2\n", data->linear_accel_xyz_ms2.x, data->linear_accel_xyz_ms2.y, data->linear_accel_xyz_ms2.z);

    // Print gravity data
    printf("\nGravity Data:\n");
    printf("X: %.2f m/s^2\n", data->gravity_x_ms2);
    printf("Y: %.2f m/s^2\n", data->gravity_y_ms2);
    printf("Z: %.2f m/s^2\n", data->gravity_z_ms2);
    printf("XYZ: [%.2f, %.2f, %.2f] m/s^2\n", data->gravity_xyz_ms2.x, data->gravity_xyz_ms2.y, data->gravity_xyz_ms2.z);
}

