/*
 * telemetry_manager.c
 *
 *  Created on: Jul 11, 2024
 *      Author: Yassine DEHHANI
 */


#include "telemetry_manager.h"

sensor_init_retval init_functions[] = {
    //ASM330LHH_Init,
    //BME680_Init,
    BMI323_Init,
    //BNO055_Init,
    //GPS_Init,
    //LIS2MDLTR_Init,
    //MS560702BA03_Init
};

const char *sensor_names[] = {
    //"ASM330LHH",
    //"BME680",
    "BMI323",
    //"BNO055",
    //"GPS",
    //"LIS2MDLTR",
    // "MS560702BA03"
};

telemetry_init_status SensorManager_Init(void) {

    size_t num_sensors = sizeof(init_functions) / sizeof(init_functions[0]);
    bool all_success = true;
    bool any_success = false;

    for (size_t i = 0; i < num_sensors; ++i) {
        bool status = init_functions[i]();
        if (!status) {
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
//    ASM330LHH_ReadData(&data->asm330lhh_data);
//    BME680_ReadData(&data->bme680_data);
//    BMI323_ReadData(&data->bmi323_data);
//    BNO055_ReadData(&data->bno055_data);
//    GPS_ReadData(&data->gps_data);
//    LIS2MDLTR_ReadData(&data->lis2mdltr_data);
//    MS560702BA03_ReadData(&data->ms560702ba03_data);
}
