/*
 * telemetry_manager.h
 *
 *  Created on: Sep 25, 2024
 *      Author: Utilisateur
 */

#ifndef INC_DATA_MANAGEMENT_TELEMETRY_MANAGER_H_
#define INC_DATA_MANAGEMENT_TELEMETRY_MANAGER_H_


#include "DRIVERS/BNO055/bno055_main.h"
#include "DRIVERS/MS5607/MS5607.h"


// Define the structure
typedef struct {
    int8_t (*init_function)(void);
    const char *sensor_name;
} sensors_init_t;

typedef enum {
    TELEMETRY_INIT_SUCCESS,         /*!< Initialization Successful of all the sensors */
    TELEMETRY_INIT_PARTIAL_SUCCESS, /*!< Initialization error on some of the sensors */
    TELEMETRY_INIT_FAILURE          /*!< Initialization error all the sensors */
}telemetry_init_status;

typedef struct {
    // Define a structure to hold all sensor data
//	IMU_6_Axis_Data asm330lhh_data;
//	Barometer_4_Axis bme680_data;
//	IMU_6_Axis_Data bmi323_data;
	AHRS_9_Axis_Data bno055_data;
//	GPS_t gps_data;
//    IMU_6_Axis_Data lis2mdltr_data;
    Barometer_2_Axis ms5607_data;
} TelemetryData;


telemetry_init_status SensorManager_Init(void);


#endif /* INC_DATA_MANAGEMENT_TELEMETRY_MANAGER_H_ */
