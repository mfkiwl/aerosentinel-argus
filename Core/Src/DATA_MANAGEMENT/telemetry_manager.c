/*
 * telemetry_manager.c
 *
 *  Created on: Sep 25, 2024
 *      Author: Utilisateur
 */


#include "DATA_MANAGEMENT/telemetry_manager.h"


TelemetryData telemetry;

// Define the array of sensors with their initialization functions and names
sensors_init_t sensors[] = {
    {BNO055_Init, "BNO055"},
    {MS5607_Init, "MS5607"},
    {ADXL375_Init, "ADXL375"},
	{MPL3115A2_Init, "MPL3115A2"},
	{BNO086_Init, "BNO086"}
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
	telemetry.bno055_data = bno_read_fusion_data();
    telemetry.ms5607_data = MS5607_ReadData();
    telemetry.adxl375_data = get_high_g_acceleration();
    telemetry.mpl311_data = mpl311_read_data();


}

void TestTelemetry(){
	for(int i = 0; i < 50 ; i++){


        //printf("// --------------------------------------------- // \n");

	// Sensor Data Read
	SensorManager_UpdateData(&telemetry);

	// Sensor Data Print

	bno055_print_fusion_data(&telemetry.bno055_data);
	ms5607_print_barometer_data(&telemetry.ms5607_data);
	adxl375_print_highG_data(&telemetry.adxl375_data);
	mpl311_print_altimetry(&telemetry.mpl311_data);

	//printf("// --------------------------------------------- // \n");

	//DELAY BETWEEN READINGS TO DESIGN CORRECTLY
    HAL_Delay(250);
    	}
}

