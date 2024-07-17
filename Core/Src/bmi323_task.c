#include <stdio.h>
#include <string.h>
#include <math.h>
#include "common_porting.h"
//#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "bmi323_task.h"
#include "DRIVERS_H/BMI323/bmi323.h"

#if defined(USE_BMI323)

extern volatile uint8_t int1_flag;
extern volatile uint8_t int2_flag;

struct bmi3_dev bmi323dev;
uint8_t bmi323_dev_addr;

#if defined(ACC_ONLY) || defined(GYRO_ONLY) || defined(ACC_GYRO)
struct bmi3_sensor_data sensor_data = { 0 };
#if defined(STEP_COUNTER)
struct bmi3_sensor_data sensor_data_stepcounter = { 0 };
#endif
#elif defined(WRIST_WEAR_WAKE_UP)
struct bmi3_sensor_data sensor_data_AG[4] = { 0 };
#elif defined(ACC_GYRO)
struct bmi3_sensor_data sensor_data_AG[3] = { 0 };
#endif
volatile uint16_t bmi323_fifo_ready = 0;
#if defined(FIFO_POLL) || defined(FIFO_WM_INT) || defined(STEP_COUNTER)
//TimerHandle_t BMI323_Timer_Handler;
/* Variable to index bytes */
uint16_t idx = 0;
/* Number of bytes of FIFO data */
uint8_t fifo_data[2048] = {0};
/* Array of accelerometer frames -> Total bytes =
150 * (6 axes bytes + 1 header byte) = 1050 bytes*/
struct bmi3_fifo_sens_axes_data fifo_accel_data[400] = { {0} };
struct bmi3_fifo_sens_axes_data fifo_gyr_data[400] = { {0} };
struct bmi3_fifo_temperature_data fifo_temp_data[266];
/* Variable to store temperature */
float temperature_value;


/* Initialize FIFO frame structure */
struct bmi3_fifo_frame fifoframe = {0};
#endif

/******************************************************************************/
/*!               User interface functions                                    */

/*!
 * @brief This API prints the execution status
 */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI3_OK:

            /*! Do nothing */
            break;

        case BMI3_E_NULL_PTR:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI3_E_COM_FAIL:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI3_E_DEV_NOT_FOUND:
            printf("%s\t", api_name);
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI3_E_INVALID_SENSOR:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INT_PIN:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI3_E_ACC_INVALID_CFG:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x20\r\n",
                rslt);
            break;

        case BMI3_E_GYRO_INVALID_CFG:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x21\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INPUT:
            printf("%s\t", api_name);
            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI3_E_INVALID_STATUS:
            printf("%s\t", api_name);
            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI3_E_DATA_RDY_INT_FAILED:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_FOC_POSITION:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_ST_SELECTION:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid self-test selection error. It occurs when there is an invalid precondition" "settings such as alternate accelerometer and gyroscope enable bits, accelerometer mode and output data rate\r\n",
                rslt);
            break;

        case BMI3_E_OUT_OF_RANGE:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Out of range error. It occurs when the range exceeds the maximum range for accel while performing FOC\r\n",
                rslt);
            break;

        case BMI3_E_FEATURE_ENGINE_STATUS:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Feature engine status error. It occurs when the feature engine enable mask is not set\r\n",
                rslt);
            break;

        default:
            printf("%s\t", api_name);
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi3_interface_init(struct bmi3_dev *bmi, uint8_t intf)
{
	int8_t rslt = BMI3_OK;
	
	/* Bus configuration : I2C */
	if (intf == BMI3_I2C_INTF)
	{
		printf("I2C Interface \n");

		/* To initialize the user I2C function */
		bmi323_dev_addr = BMI3_ADDR_I2C_SEC;
		bmi->intf = BMI3_I2C_INTF;
		bmi->read = (bmi3_read_fptr_t)BMI323_SensorAPI_I2Cx_Read;
		bmi->write = (bmi3_write_fptr_t)BMI323_SensorAPI_I2Cx_Write;
	}
	/* Bus configuration : SPI */
	else if (intf == BMI3_SPI_INTF)
	{
		printf("SPI Interface \n");
		bmi->intf = BMI3_SPI_INTF;
		bmi->read = (bmi3_read_fptr_t)SensorAPI_SPIx_Read;
		bmi->write = (bmi3_write_fptr_t)SensorAPI_SPIx_Write;
	}


	/* Assign device address to interface pointer */
	bmi->intf_ptr = &bmi323_dev_addr;

	/* Configure delay in microseconds */
	bmi->delay_us = bst_delay_us;

	/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
	bmi->read_write_len = READ_WRITE_LEN;

	return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

int8_t Open_BMI323_ACC(struct bmi3_dev *dev)
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
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
	#endif
		

		config.cfg.acc.odr      = BMI3_ACC_ODR_50HZ;

		/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
		config.cfg.acc.range     = BMI3_ACC_RANGE_4G;
		
		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK) 
		{
			printf("Open ACC failed, rslt=%d\r\n", rslt);
		} 
		else
		{
			printf("Open ACC set successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK) 
			{
				printf("Get ACC configuration successful\r\n");
				printf("acc_mode = %d\r\n", config.cfg.acc.acc_mode);
				printf("bwp = %d\r\n", config.cfg.acc.bwp);
				printf("odr = %d\r\n", config.cfg.acc.odr);
				printf("Range = %d\r\n", config.cfg.acc.range);
				printf("avg_num = %d\r\n", config.cfg.acc.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_ACC(struct bmi3_dev *dev)
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
			printf("Close ACC failed, rslt=%d\r\n", rslt);
		} 
		else
		{
			printf("Open ACC successfully\r\n");
		}
	}
	bmi3_error_codes_print_result("Close_BMI323_ACC",rslt);

	return rslt;
}

int8_t Open_BMI323_GYRO(struct bmi3_dev *dev)
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
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
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
			printf("Open GYRO failed\r\n");
		} 
		else
		{
			printf("Open GYRO successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK) 
			{
				printf("Get BMI2_GYRO Configuration successful\r\n");
				printf("gyr_mode = %d\r\n", config.cfg.gyr.gyr_mode);
				printf("ODR = %d\r\n", config.cfg.gyr.odr);
				printf("Range = %d\r\n", config.cfg.gyr.range);
				printf("bwp = %d\r\n", config.cfg.gyr.bwp);
				printf("avg_num = %d\r\n", config.cfg.gyr.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_GYRO(struct bmi3_dev *dev)
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
			printf("Close GYRO failed, rslt=%d\r\n", rslt);
		} 
		else
		{
			printf("Open GYRO successfully\r\n");
		}
	}
	bmi3_error_codes_print_result("Close_BMI323_GYRO",rslt);

	return rslt;
}

#if defined(ACC_RDRY_INT)
int8_t Open_BMI323_ACC_DRDY_INT(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	struct bmi3_map_int map_int = { 0 };
	struct bmi3_int_pin_config int_cfg;

	map_int.acc_drdy_int = BMI3_INT1;

	/* Map data ready interrupt to interrupt pin. */
	rslt = bmi323_map_interrupt(map_int, dev);
	bmi3_error_codes_print_result("Open_BMI323_ACC_DRDY_INT", rslt);
	
	bmi3_get_int_pin_config(&int_cfg, dev);

	int_cfg.pin_type = BMI3_INT1;
	int_cfg.pin_cfg[0].lvl = BMI3_INT_ACTIVE_HIGH;/*Config INT1 rising edge trigging*/
	int_cfg.pin_cfg[0].od = BMI3_INT_PUSH_PULL;
	int_cfg.pin_cfg[0].output_en= BMI3_INT_OUTPUT_ENABLE;

	rslt = bmi3_set_int_pin_config(&int_cfg, dev);
	bmi3_error_codes_print_result("Open_BMI323_ACC_DRDY_INT", rslt);

	return rslt;
}

int8_t Close_BMI323_ACC_RDRY_INT(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;

	struct bmi3_map_int map_int = { 0 };
	struct bmi3_int_pin_config int_cfg;

	map_int.acc_drdy_int = BMI3_INT_NONE;

	/* Map data ready interrupt to interrupt pin. */
	rslt = bmi323_map_interrupt(map_int, dev);
	bmi3_error_codes_print_result("Close_BMI323_ACC_RDRY_INT", rslt);
	
	return rslt;
}
#endif

#if defined(GYRO_RDRY_INT)
int8_t Open_BMI323_GYRO_DRDY_INT(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	struct bmi3_map_int map_int = { 0 };
	struct bmi3_int_pin_config int_cfg;

	map_int.gyr_drdy_int = BMI3_INT1;

	/* Map data ready interrupt to interrupt pin. */
	rslt = bmi323_map_interrupt(map_int, dev);
	bmi3_error_codes_print_result("Open_BMI323_GYRO_DRDY_INT", rslt);
	
	bmi3_get_int_pin_config(&int_cfg, dev);

	int_cfg.pin_type = BMI3_INT1;
	int_cfg.pin_cfg[0].lvl = BMI3_INT_ACTIVE_HIGH;/*Config INT1 rising edge trigging*/
	int_cfg.pin_cfg[0].od = BMI3_INT_PUSH_PULL;
	int_cfg.pin_cfg[0].output_en= BMI3_INT_OUTPUT_ENABLE;

	rslt = bmi3_set_int_pin_config(&int_cfg, dev);
	bmi3_error_codes_print_result("Open_BMI323_GYRO_DRDY_INT", rslt);

	return rslt;
}

int8_t Close_BMI323_GYRO_RDRY_INT(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;

	struct bmi3_map_int map_int = { 0 };
	struct bmi3_int_pin_config int_cfg;

	map_int.gyr_drdy_int = BMI3_INT_NONE;

	/* Map data ready interrupt to interrupt pin. */
	rslt = bmi323_map_interrupt(map_int, dev);
	bmi3_error_codes_print_result("Close_BMI323_GYRO_RDRY_INT", rslt);
	
	return rslt;
}
#endif


#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
int8_t Open_BMI323_FIFO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;

	#if defined(FIFO_WM_INT)
	struct bmi3_int_pin_config int_cfg;
	struct bmi3_map_int map_int = { 0 };
	#endif
	/* Array to define set FIFO flush */
    	uint8_t data[2] = { BMI323_ENABLE, 0 };

	/* Set the FIFO flush in FIFO control register to clear the FIFO data */
    	rslt = bmi323_set_regs(BMI3_REG_FIFO_CTRL, data, 2, dev);
    	bmi3_error_codes_print_result("bmi323_set_regs", rslt);

	/* Clear FIFO configuration register */
	rslt = bmi3_set_fifo_config(BMI3_FIFO_ALL_EN, BMI3_DISABLE, dev);
	bmi3_error_codes_print_result("bmi323_set_fifo_config", rslt);

	/*Example: 100Hz ODR, read data per second*/
	/* Set FIFO configuration by enabling accelerometer and gyroscope*/
	printf("FIFO Header is disabled\r\n");

	#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
		#if defined(ACC_ONLY)
		/* Set the water-mark level */
		rslt = bmi3_set_fifo_wm(150, dev);//6*50=300 bytes, 50HZ ODR, read data every second
		if (rslt != BMI3_OK) 
		{
			printf("bmi3_set_fifo_wm error, error code: %d\r\n", rslt);
		}
		rslt = bmi3_set_fifo_config(BMI3_FIFO_ACC_EN , BMI3_ENABLE, dev);
		#elif defined(GYRO_ONLY)
		/* Set the water-mark level */
		rslt = bmi3_set_fifo_wm(150, dev);//6*50=300 bytes, 50HZ ODR, read data every second
		if (rslt != BMI3_OK) 
		{
			printf("bmi3_set_fifo_wm error, error code: %d\r\n", rslt);
		}
		rslt = bmi3_set_fifo_config(BMI3_FIFO_GYR_EN , BMI3_ENABLE, dev);
		#elif defined(ACC_GYRO)
		/* Set the water-mark level */
		rslt = bmi3_set_fifo_wm(300, dev);//12*50=600, 100HZ ODR, read data every second
		if (rslt != BMI3_OK) 
		{
			printf("bmi3_set_fifo_wm error, error code: %d\r\n", rslt);
		}
		rslt = bmi3_set_fifo_config(BMI3_FIFO_ACC_EN | BMI3_FIFO_GYR_EN , BMI3_ENABLE, dev);
		#endif
		if (rslt != BMI3_OK)
		{
			printf("Set fifo config failed\r\n");
		}
	#endif

	#if defined(FIFO_WM_INT)
	bmi3_get_int_pin_config(&int_cfg, dev);
	
	int_cfg.pin_type = BMI3_INT2;
	int_cfg.pin_cfg[1].lvl = BMI3_INT_ACTIVE_HIGH;
	int_cfg.pin_cfg[1].od = BMI3_INT_PUSH_PULL;;
	int_cfg.pin_cfg[1].output_en= BMI3_INT_OUTPUT_ENABLE;

	bmi3_set_int_pin_config(&int_cfg, dev);

	/* Map the FIFO water-mark interrupt to INT1 */
	/* Note: User can map the interrupt to INT1 or INT2 */
	map_int.fifo_watermark_int = BMI3_INT2;

	/* Map the interrupt configuration */
	rslt = bmi323_map_interrupt(map_int, dev);
	bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);
	#endif

	/* Update FIFO structure */
	fifoframe.data = fifo_data;
	
	return rslt;
}
#endif

int8_t Close_BMI323_FIFO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	
	/* Clear FIFO configuration register */
	rslt = bmi3_set_fifo_config(BMI3_FIFO_ALL_EN, BMI3_DISABLE, dev);
	if (rslt != BMI3_OK) 
	{
		printf("Clear FIFO configuration register error, error code: %d\r\n", rslt);
		return rslt;
	}
	
	return rslt;
}

int8_t Open_BMI323_STEP_COUNTER(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };
	/* Feature enable initialization. */
    	struct bmi3_feature_enable feature = { 0 };

	config.type = BMI3_STEP_COUNTER;

	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	bmi3_error_codes_print_result("Get sensor config", rslt);
	
	if (rslt == BMI323_OK)
	{
		/* Function to select the context feature configurations.
		* For smart phone: BMI323_SMART_PHONE_SEL
		* For wearables  : BMI323_WEARABLE_SEL
		* For hearables  : BMI323_HEARABLE_SEL */
		rslt = bmi323_context_switch_selection(BMI323_WEARABLE_SEL, dev);
		bmi3_error_codes_print_result("context switch selection", rslt);

		if (rslt == BMI323_OK)
		{
			/* Get default configurations for the type of feature selected. */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			bmi3_error_codes_print_result("Get sensor config", rslt);

			printf("Step counter wearable configurations\n");
			printf("Watermark level = %x\n", config.cfg.step_counter.watermark_level);
			printf("Reset counter = %x\n", config.cfg.step_counter.reset_counter);
			printf("env_min_dist_up: %d\n", config.cfg.step_counter.env_min_dist_up);
			printf("env_coef_up: %d\n", config.cfg.step_counter.env_coef_up);
			printf("env_min_dist_down: %d\n", config.cfg.step_counter.env_min_dist_down);
			printf("env_coef_down: %d\n", config.cfg.step_counter.env_coef_down);
			printf("eean_val_decay: %d\n", config.cfg.step_counter.mean_val_decay);
			printf("eean_step_dur: %d\n", config.cfg.step_counter.mean_step_dur);
			printf("step_buffer_size: %d\n", config.cfg.step_counter.step_buffer_size);
			printf("filter_cascade_enabled: %d\n", config.cfg.step_counter.filter_cascade_enabled);
			printf("step_counter_increment: %d\n", config.cfg.step_counter.step_counter_increment);
			printf("peak_duration_min_walking: %d\n", config.cfg.step_counter.peak_duration_min_walking);
			printf("peak_duration_min_running: %d\n", config.cfg.step_counter.peak_duration_min_running);
			printf("activity_detection_factor: %d\n", config.cfg.step_counter.activity_detection_factor);
			printf("activity_detection_thres: %d\n", config.cfg.step_counter.activity_detection_thres);
			printf("step_duration_max: %d\n", config.cfg.step_counter.step_duration_max);
			printf("step_duration_window: %d\n", config.cfg.step_counter.step_duration_window);
			printf("step_duration_pp_enabled: %d\n", config.cfg.step_counter.step_duration_pp_enabled);
			printf("step_duration_thres: %d\n", config.cfg.step_counter.step_duration_thres);
			printf("mean_crossing_pp_enabled: %d\n", config.cfg.step_counter.mean_crossing_pp_enabled);
			printf("mcr_threshold: %d\n", config.cfg.step_counter.mcr_threshold);
			printf("sc_12_res: %d\n", config.cfg.step_counter.sc_12_res);

			if (rslt == BMI323_OK)
			{
				/* Enable water-mark level for to get interrupt after 20 step counts. */
				config.cfg.step_counter.watermark_level = 1;

				/* Set new configurations. */
				rslt = bmi323_set_sensor_config(&config, 1, dev);
				bmi3_error_codes_print_result("Set sensor config", rslt);

				if (rslt == BMI323_OK)
				{
					/* Get default configurations for the type of feature selected. */
					rslt = bmi323_get_sensor_config(&config, 1, dev);
					bmi3_error_codes_print_result("Get sensor config", rslt);

					printf("Step counter watermark level is %d\n", config.cfg.step_counter.watermark_level);
				}
			}
		}
	}

	feature.step_counter_en = BMI323_ENABLE;

	/* Enable the selected sensors. */
	rslt = bmi323_select_sensor(&feature, dev);
	bmi3_error_codes_print_result("Sensor enable", rslt);
	
	return rslt;
}

int8_t Close_BMI323_STEP_COUNTER(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Feature enable initialization. */
    	struct bmi3_feature_enable feature = { 0 };
	feature.step_counter_en = BMI323_DISABLE;

	/* Enable the selected sensors. */
	rslt = bmi323_select_sensor(&feature, dev);
	bmi3_error_codes_print_result("Sensor disable", rslt);
	return rslt;
}



void BMI323_Print_ALLRegs(struct bmi3_dev *dev)
{
	uint8_t value;

	uint8_t reg_addr;

	for(reg_addr = 0; reg_addr < 0x80; reg_addr++)
	{
		bmi3_get_regs(reg_addr, &value, 1, dev);
		printf("0x%02X, value=0x%02X\r\n", reg_addr, value);
	}
}

int8_t Init_BMI323(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	uint8_t chipid;

	/* Interface reference is given as a parameter
	* For I2C : BMI3_I2C_INTF
	* For SPI : BMI3_SPI_INTF
	*/
	#if defined(USE_I2C_INTERFACE)
	rslt = bmi3_interface_init(dev, BMI3_I2C_INTF);
	#elif defined(USE_SPI_INTERFACE)
	rslt = bmi3_interface_init(dev, BMI3_SPI_INTF);
	#endif
	bmi3_error_codes_print_result("bmi3_interface_init",rslt);

	//bmi3_soft_reset(dev);
	//HAL_Delay(100);
	bst_delay_us(100000, dev->intf_ptr);

	/* Initialize bmi323. */
	rslt = bmi323_init(dev);
	printf("bmi323_init: 0x%02X\n",rslt);

	if (rslt != BMI3_OK)
	{
		printf("bmi323_init() failed, error code: %d\r\n", rslt);
		return rslt;
	}
	else
	{
		printf("BMI323 initialized successfully\r\n");
	}

	rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, &chipid, 1, dev);
	if (rslt != BMI3_OK) 
	{
		printf("read chip ID failed, error code: %d\r\n", rslt);
		return rslt;
	}

	printf("Chip ID:0x%02x\r\n", chipid);

	//rslt = bmi323_get_config_version(&ver_major, &ver_minor, dev);
	//printf("The firmware version: v%d.%d\r\n", ver_major, ver_minor);

	/*Axis mapping according phisical structure*/
#if 0//For axis remap
	/* Strusture instance of axes remap */
	struct bmi3_axes_remap remap = { 0 };

	rslt = bmi323_get_remap_axes(&remap, &dev);
	bmi3_error_codes_print_result("get_remap_axes", rslt);

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
		bmi3_error_codes_print_result("set_remap_axes", rslt);

		if (rslt != BMI323_OK)
		{
			printf("bmi323_set_remap_axes() failed\r\n");
		}
	}
	else
	{
		printf("bmi323_get_remap_axes() failed\r\n");
	}

#endif
#if defined(FOC)
//bmi3_perform_accel_foc();
//bmi3_perform_gyro_foc(dev)
#endif
	struct bmi3_st_result st_result_status = { 0 };
#if defined(ACC_SELFTEST)
	rslt = bmi323_perform_self_test(BMI3_ST_ACCEL_ONLY, &st_result_status, dev);
	bmi3_error_codes_print_result("Perform_self_test", rslt);

	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
	{
		printf("ACC self-test is successfully completed \n");
	}
	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
	{
		printf("Self-test is not successfully completed\n");

		switch (st_result_status.self_test_err_rslt)
		{
			case BMI3_SC_ST_ABORTED_MASK:
				printf("SC_ST_ABORTED\n");
				break;
			case BMI3_ST_IGNORED_MASK:
				printf("BMI323_ST_IGNORED\n");
				break;
			case BMI3_SC_ST_PRECON_ERR_MASK:
				printf("BMI323_SC_ST_PRECON_ERR\n");
				break;
			case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
				printf("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
				break;
			default:
				break;
		}
	}

	printf("Result of acc_x_axis is %d\n", st_result_status.acc_sens_x_ok);
	printf("Result of acc_y_axis is %d\n", st_result_status.acc_sens_y_ok);
	printf("Result of acc_z_axis is %d\n", st_result_status.acc_sens_z_ok);
	printf("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
	printf("Result of ST_result is %d\n", st_result_status.self_test_rslt);

	for(;;)
	{
		;
	}
#elif defined(GYRO_SELFTEST)
	rslt = bmi323_perform_self_test(BMI3_ST_GYRO_ONLY, &st_result_status, dev);
	bmi3_error_codes_print_result("Perform_self_test", rslt);

	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
	{
		printf("ACC self-test is successfully completed \n");
	}
	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
	{
		printf("Self-test is not successfully completed\n");

		switch (st_result_status.self_test_err_rslt)
		{
			case BMI3_SC_ST_ABORTED_MASK:
				printf("SC_ST_ABORTED\n");
				break;
			case BMI3_ST_IGNORED_MASK:
				printf("BMI323_ST_IGNORED\n");
				break;
			case BMI3_SC_ST_PRECON_ERR_MASK:
				printf("BMI323_SC_ST_PRECON_ERR\n");
				break;
			case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
				printf("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
				break;
			default:
				break;
		}
	}

	printf("Result of gyr_x_axis is %d\n", st_result_status.gyr_sens_x_ok);
	printf("Result of gyr_y_axis is %d\n", st_result_status.gyr_sens_y_ok);
	printf("Result of gyr_z_axis is %d\n", st_result_status.gyr_sens_z_ok);
	printf("Result of gyr_drive_ok is %d\n", st_result_status.gyr_drive_ok);
	printf("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
	printf("Result of ST_result is %d\n", st_result_status.self_test_rslt);

	for(;;)
	{
		;
	}
#elif defined(ACC_GYRO_SELFTEST)
	rslt = bmi323_perform_self_test(BMI3_ST_BOTH_ACC_GYR, &st_result_status, dev);
	bmi3_error_codes_print_result("Perform_self_test", rslt);

	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
	{
		printf("ACC self-test is successfully completed \n");
	}
	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
	{
		printf("Self-test is not successfully completed\n");

		switch (st_result_status.self_test_err_rslt)
		{
			case BMI3_SC_ST_ABORTED_MASK:
				printf("SC_ST_ABORTED\n");
				break;
			case BMI3_ST_IGNORED_MASK:
				printf("BMI323_ST_IGNORED\n");
				break;
			case BMI3_SC_ST_PRECON_ERR_MASK:
				printf("BMI323_SC_ST_PRECON_ERR\n");
				break;
			case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
				printf("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
				break;
			default:
				break;
		}
	}

	printf("Result of acc_x_axis is %d\n", st_result_status.acc_sens_x_ok);
	printf("Result of acc_y_axis is %d\n", st_result_status.acc_sens_y_ok);
	printf("Result of acc_z_axis is %d\n", st_result_status.acc_sens_z_ok);
	printf("Result of gyr_x_axis is %d\n", st_result_status.gyr_sens_x_ok);
	printf("Result of gyr_y_axis is %d\n", st_result_status.gyr_sens_y_ok);
	printf("Result of gyr_z_axis is %d\n", st_result_status.gyr_sens_z_ok);
	printf("Result of gyr_drive_ok is %d\n", st_result_status.gyr_drive_ok);
	printf("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
	printf("Result of ST_result is %d\n", st_result_status.self_test_rslt);
	for(;;)
	{
		;
	}	
#elif defined(CRT)
	uint8_t sens_sel[2] = { BMI3_ACCEL, BMI3_GYRO };

	rslt = bmi2_sensor_enable(&sens_sel[0], 1, dev);    
	bmi2_perform_accel_self_test(rslt);    
	
	rslt = bmi2_sensor_disable(&sens_sel[1], 1, dev);    
	bmi2_perform_accel_self_test(rslt);    
	
	sensor_data.type = BMI2_ACCEL;    
	dev.delay_us(100000);    
	printf("before CRT Accel x,y,z values\r\n");

	/* read the accel data before CRT*/    
	rslt = bmi2_get_sensor_data(&sensor_data, 1, dev);    
	bmi2_perform_accel_self_test(rslt);    
	printf("X axes: %d, Y axes: %d, Z axes: %d\r\n", sensor_data.sens_data.acc.x, sensor_data.sens_data.acc.y, sensor_data.sens_data.acc.z );

	/*brief This API is to run the CRT process*/    
	rslt = bmi2_do_crt(dev);    
	bmi2_perform_accel_self_test(rslt);    
	if(rslt == BMI3_OK)    
	{    	
		printf("After CRT Accel x,y,z values\r\n");
		/* read the accel data after CRT*/    	
		rslt = bmi2_get_sensor_data(&sensor_data, 1, dev);    	
		bmi2_perform_accel_self_test(rslt);    	
		printf("X axes: %d, Y axes: %d, Z axes: %d\r\n",sensor_data.sens_data.acc.x, sensor_data.sens_data.acc.y, sensor_data.sens_data.acc.z );
	}
#endif

	#if defined(ACC_ONLY)
	Open_BMI323_ACC(dev);
	Close_BMI323_GYRO(dev);
	#elif defined(GYRO_ONLY)
	Close_BMI323_ACC(dev);
	Open_BMI323_GYRO(dev);
	#elif defined(ACC_GYRO)
	Open_BMI323_ACC(dev);
	Open_BMI323_GYRO(dev);
	#endif

	#if defined(STEP_COUNTER)
	Open_BMI323_STEP_COUNTER(dev);
	#endif

	#if defined(FIFO_POLL)
	Open_BMI323_FIFO(dev);
	#elif defined(FIFO_WM_INT)
	Open_BMI323_FIFO(dev);
	Enable_MCU_INT2_Pin();
	#endif

	#if defined(ACC_RDRY_INT)
	Open_BMI323_ACC_DRDY_INT(dev);
	#endif

	#if defined(ACC_RDRY_INT)
	Open_BMI323_GYRO_DRDY_INT(dev);
	#endif

	#if defined(ACC_RDRY_INT) || defined(GYRO_RDRY_INT)
	Enable_MCU_INT1_Pin();
	#endif

	#if !defined(FIFO_WM_INT)
	Disable_MCU_INT2_Pin();
	#endif

	return rslt;
}

//#if defined(FIFO_POLL) || defined(STEP_COUNTER)
//void BMI323_Timer_Callback(TimerHandle_t xTimer)
//{
//	bmi323_fifo_ready = 1;
//}
//#endif

void StartBMI323Task(void const * argument)
{
	int8_t rslt = BMI3_OK;
	struct bmi3_dev *dev;
	dev = &bmi323dev;

#if defined(ACC_RDRY_INT) || defined(GYRO_RDRY_INT)
	osThreadId BMI323InterruptTaskHandle;
	osThreadDef(BMI323INTERRUPTTask, StartBMI323InterruptTask, osPriorityNormal, 0, 256);
  	BMI323InterruptTaskHandle = osThreadCreate(osThread(BMI323INTERRUPTTask), NULL);
#endif

	Init_BMI323(dev);
	
//	#if defined(FIFO_POLL)
//	BMI323_Timer_Handler = xTimerCreate("FIFOReadTimer", 1010, pdTRUE, (void *)1, BMI323_Timer_Callback);
//	if(BMI323_Timer_Handler != NULL)
//	{
//		xTimerStart(BMI323_Timer_Handler, 1000);
//	}
//	#endif

	for(;;)
	{
		if(bmi323_fifo_ready == 1)
		{	
			bmi323_fifo_ready = 0;
			#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
			memset(fifo_data, 0, sizeof(fifo_data));
			fifoframe.length = 0;
			
			/* Update FIFO structure */
			fifoframe.data = fifo_data;
			
			rslt = bmi323_get_fifo_length(&fifoframe.available_fifo_len, dev);
	            	bmi3_error_codes_print_result("bmi323_get_fifo_length", rslt);

	            	//fifoframe.length = (uint16_t)(fifoframe.available_fifo_len * 2) + dev->dummy_byte;
			fifoframe.length = (uint16_t)(fifoframe.available_fifo_len * 2) + 2;
			printf("Fifo length=%d\r\n", fifoframe.length);
			if(fifoframe.length > 0)
			{				
				/* Read FIFO data */
				rslt = bmi323_read_fifo_data(&fifoframe, dev);
				bmi3_error_codes_print_result("bmi323_read_fifo_data", rslt);
	
				if (rslt == BMI323_OK) 
				{
					printf("Read %d bytes from fifo\r\n", fifoframe.length);

					#if defined(ACC_ONLY) || defined(ACC_GYRO)
					
					/* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
					rslt = bmi323_extract_accel(fifo_accel_data, &fifoframe, dev);
					printf("Parsed accelerometer data frames: %d\r\n", fifoframe.avail_fifo_accel_frames);
					if (rslt == BMI3_OK) 
					{
						/* Print the parsed accelerometer data from the FIFO buffer */
						for(idx = 0 ; idx < fifoframe.avail_fifo_accel_frames ; idx++) 
						{
							//printf("ACCEL[%d] X : %d , Y : %d , Z : %d\r\n", idx , fifo_accel_data[idx].x, fifo_accel_data[idx].y, fifo_accel_data[idx].z);
						}
					}
					#endif
					#if defined(GYRO_ONLY) || defined(ACC_GYRO)
					
					/* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
					rslt = bmi323_extract_gyro(fifo_gyr_data, &fifoframe, dev);
					printf("Parsed gyroscope data frames: %d\r\n", fifoframe.avail_fifo_gyro_frames);
					if (rslt == BMI3_OK) 
					{
						/* Print the parsed accelerometer data from the FIFO buffer */
						for(idx = 0 ; idx < fifoframe.avail_fifo_gyro_frames ; idx++) 
						{
							//printf("GYRO[%d] X : %d , Y : %d , Z : %d\r\n", idx , fifo_gyr_data[idx].x, fifo_gyr_data[idx].y, fifo_gyr_data[idx].z);
						}
					}
					#endif
				}
			}
			#endif
			#if defined(STEP_COUNTER)
			sensor_data_stepcounter.type = BMI323_STEP_COUNTER;
			rslt = bmi323_get_sensor_data(&sensor_data_stepcounter, 1, dev);
			if (rslt == BMI3_OK) 
			{
				printf("Step count = %u\r\n", sensor_data_stepcounter.sens_data.step_counter_output);
			}
			#endif
		}
	}
}

#if defined(ACC_RDRY_INT) || defined(GYRO_RDRY_INT)
void StartBMI323InterruptTask(void const * argument)
{
	int8_t rslt = BMI3_OK;
	struct bmi3_dev *dev;
	float x = 0, y = 0, z = 0;
	
	dev = &bmi323dev;
	
	/* Variable to get wrist gesture status */
	uint16_t int_status = 0;
	
	for(;;)
	{
		if(int1_flag == 1)
        	{
			/* Check the interrupt status of the wrist gesture */
			
			rslt = bmi323_get_int1_status(&int_status, dev);
			bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);
			if (rslt == BMI3_OK)
			{
				printf("Get int status=0x%04X\r\n", int_status);
				if (int_status & BMI3_INT_STATUS_ACC_DRDY)
				{
					printf("acc data ready detected\r\n");
					/* Get accelerometer data for x, y and z axis. */
		                    rslt = bmi323_get_sensor_data(&sensor_data, 1, dev);
		                    bmi3_error_codes_print_result("bmi323_get_sensor_data",rslt);
		                    printf("\nAcc_X = %d\t", sensor_data.sens_data.acc.x);
		                    printf("Acc_Y = %d\t", sensor_data.sens_data.acc.y);
		                    printf("Acc_Z = %d\r\n", sensor_data.sens_data.acc.z);

		                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
		                    x = lsb_to_mps2(sensor_data.sens_data.acc.x, 2, dev->resolution);
		                    y = lsb_to_mps2(sensor_data.sens_data.acc.y, 2, dev->resolution);
		                    z = lsb_to_mps2(sensor_data.sens_data.acc.z, 2, dev->resolution);

		                    /* Print the data in m/s2. */
		                    printf("\nAcc_ms2_X = %4.2f, Acc_ms2_Y = %4.2f, Acc_ms2_Z = %4.2f\n", x, y, z);
				}
				if (int_status & BMI3_INT_STATUS_GYR_DRDY)
				{
					printf("gyro data ready detected\r\n");
					
					/* Get gyro data for x, y and z axis. */
					rslt = bmi323_get_sensor_data(&sensor_data, 1, dev);
					bmi3_error_codes_print_result("Get sensor data", rslt);

					/* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
					x = lsb_to_dps(sensor_data.sens_data.gyr.x, (float)2000, dev->resolution);
					y = lsb_to_dps(sensor_data.sens_data.gyr.y, (float)2000, dev->resolution);
					z = lsb_to_dps(sensor_data.sens_data.gyr.z, (float)2000, dev->resolution);

					/* Print the data in dps. */
					printf("%d, %d, %d, %4.2f, %4.2f, %4.2f\n",
						   sensor_data.sens_data.gyr.x,
						   sensor_data.sens_data.gyr.y,
						   sensor_data.sens_data.gyr.z,
						   x,
						   y,
						   z);
				}
			}
			int1_flag = 0;
		}
	}
}
#endif
#endif
