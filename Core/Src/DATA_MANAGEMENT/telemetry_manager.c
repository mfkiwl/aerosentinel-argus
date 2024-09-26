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
    {ADXL375_Init, "ADXL375"}
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
