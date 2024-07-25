/*
 * telemetry_manager.c
 *
 *  Created on: Jul 11, 2024
 *      Author: Yassine DEHHANI
 */


#include "telemetry_manager.h"


TelemetryData telemetry;


// Define the array of sensors with their initialization functions and names
sensors_init_t sensors[] = {
    {Init_BMI323, "BMI323"},
    {BNO055_Init, "BNO055"},
    {BME680_Init, "BME680"},
    {MS5607_Init, "MS5607"},
    {GPS_Init, "ATGM336H"}
};


telemetry_init_status SensorManager_Init(void) {
    printf("Sensors Initialization routine started.\n");

    size_t num_sensors = sizeof(sensors) / sizeof(sensors[0]);
    bool all_success = true;
    bool any_success = false;

    for (size_t i = 0; i < num_sensors; ++i) {
        int8_t status = sensors[i].init_function();
        if (status != 0) {
            printf("%s initialization failed.\n", sensors[i].sensor_name);
            all_success = false;
        } else {
            printf("[%s] device found! Initialization succeeded.\n", sensors[i].sensor_name);
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
    telemetry.gps_data = GPS_Data_Reception();
    //TO IMPLEMENT
//    GPS_ReadData(&data->gps_data);

    //FOR VERSION 2 OF ARGUS
//    ASM330LHH_ReadData(&data->asm330lhh_data);
//    LIS2MDLTR_ReadData(&data->lis2mdltr_data);

}

void delay_us_func(uint32_t period)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 96; i++)
		{
			;
		}
	}
}

void TestTelemetry(){
	for(int i = 0; i < 1000 ; i++){

	// Sensor Data Read
	SensorManager_UpdateData(&telemetry);

	// Sensor Data Print
	bmi323_print_sensor_data(&telemetry.bmi323_data);
	bno055_print_fusion_data(&telemetry.bno055_data);
	ms5607_print_barometer_data(&telemetry.ms5607_data);
	bme680_print_barometer_data(&telemetry.bme680_data);
	gps_print_positionning_data(&telemetry.gps_data);

	printf("// --------------------------------------------- // \n");

	//DELAY BETWEEN READINGS TO DESIGN CORRECTLY
//    HAL_Delay(150);
//    delay_us_func(50000);  //50 ms
//    delay_us_func(100000); //100 ms
//    delay_us_func(200000); //200 ms
      delay_us_func(1000000); //1 s
    	}
}
