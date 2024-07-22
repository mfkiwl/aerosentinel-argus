/*
 * telemetry_manager.c
 *
 *  Created on: Jul 11, 2024
 *      Author: Yassine DEHHANI
 */


#include "telemetry_manager.h"


AHRS_9_Axis_Data bno055_fusion_data;
IMU_6_Axis_Data bmi323_sensor_data;

TelemetryData telemetry;

sensor_init_retval init_functions[] = {
	Init_BMI323,
	BNO055_Init,
	BME680_Init,
	MS5607_Init,
};

const char *sensor_names[] = {
	"BMI323",
	"BNO055",
	"BME680",
	"MS5607",
};

telemetry_init_status SensorManager_Init(void) {
	printf("Sensors Initialization routine started. \n");

    size_t num_sensors = sizeof(init_functions) / sizeof(init_functions[0]);
    bool all_success = true;
    bool any_success = false;

    for (size_t i = 0; i < num_sensors; ++i) {
        int8_t status = init_functions[i]();
        if (status != 0) {
            printf("%s initialization failed.\n", sensor_names[i]);
            all_success = false;
        } else {
            any_success = true;
        }
    }

    if (all_success) {
        printf("All sensors initialized successfully.\n");
        return TELEMETRY_INIT_SUCCESS;
    } else if (any_success) {
        printf("Partial initialization success.\n");
        return TELEMETRY_INIT_PARTIAL_SUCCESS;
    } else {
        printf("All sensors initialization failed.\n");
        return TELEMETRY_INIT_FAILURE;
    }
}

void SensorManager_UpdateData(TelemetryData *data) {
    // Update data from each sensor
	telemetry.bmi323_data = bmi323_data_poll();
	telemetry.bno055_data = bno_read_fusion_data();
	telemetry.bme680_data = bme680_data_poll();
    telemetry.ms5607_data = MS5607_ReadData();
//    ASM330LHH_ReadData(&data->asm330lhh_data);
//    GPS_ReadData(&data->gps_data);
//    LIS2MDLTR_ReadData(&data->lis2mdltr_data);

}

//void delay_us_func(uint32_t period)
//{
//	uint32_t i;
//
//	while(period--)
//	{
//		for(i = 0; i < 96; i++)
//		{
//			;
//		}
//	}
//}

void TestTelemetry(){
	for(int i = 0; i < 20 ; i++){

	// Sensor Data Read
	SensorManager_UpdateData(&telemetry);

	// Sensor Data Print
	bmi323_print_sensor_data(&telemetry.bmi323_data);
	bno055_print_fusion_data(&telemetry.bno055_data);
	ms5607_print_barometer_data(&telemetry.ms5607_data);
	bme680_print_barometer_data(&telemetry.bme680_data);

	printf("// --------------------------------------------- // \n");

    HAL_Delay(150);
    	}
}
