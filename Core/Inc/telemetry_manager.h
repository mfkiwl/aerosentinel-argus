/*
 * telemetry_manager.h
 *
 *  Created on: Jul 11, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_TELEMETRY_MANAGER_H_
#define INC_TELEMETRY_MANAGER_H_


#include "DRIVERS_H/BME680/bme680_main.h"
#include "DRIVERS_H/BMI323/bmi323_main.h"
#include "DRIVERS_H/BNO055/bno055_main.h"
#include "DRIVERS_H/GPS/GPS.h"
#include "DRIVERS_H/MS560702BA03/MS5607.h"
#include "DRIVERS_H/BLACKBOX/blackbox.h"






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
    float magnetic_field[3]; // X, Y, Z magnetic field in Gauss
} Magnetometer_3_Axis_Data;

typedef struct {
    // Define a structure to hold all sensor data
	IMU_6_Axis_Data asm330lhh_data;
	Barometer_4_Axis bme680_data;
	IMU_6_Axis_Data bmi323_data;
	AHRS_9_Axis_Data bno055_data;
	GPS_t gps_data;
    IMU_6_Axis_Data lis2mdltr_data;
    Barometer_2_Axis ms5607_data;
} TelemetryData;


telemetry_init_status SensorManager_Init(void);
void SensorManager_UpdateData(TelemetryData *data);
void TestTelemetry();


#endif /* INC_TELEMETRY_MANAGER_H_ */
