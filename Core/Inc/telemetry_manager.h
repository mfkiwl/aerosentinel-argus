/*
 * telemetry_manager.h
 *
 *  Created on: Jul 11, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_TELEMETRY_MANAGER_H_
#define INC_TELEMETRY_MANAGER_H_


#include "DRIVERS_H/ASM330LHH/asm330lhh_main.h"
#include "DRIVERS_H/BME680/bme680_main.h"
#include "DRIVERS_H/BMI323/bmi323_main.h"
#include "DRIVERS_H/BNO055/bno055_main.h"
#include "DRIVERS_H/GPS/GPS.h"
#include "DRIVERS_H/LIS2MDLTR/lis2mdl_main.h"
#include "DRIVERS_H/MS560702BA03/MS5607_main.h"
#include "DRIVERS_H/BLACKBOX/blackbox.h"


typedef bool (*sensor_init_retval)(void);

typedef enum {
    TELEMETRY_INIT_SUCCESS,         /*!< Initialization Successful of all the sensors */
    TELEMETRY_INIT_PARTIAL_SUCCESS, /*!< Initialization error on some of the sensors */
    TELEMETRY_INIT_FAILURE          /*!< Initialization error all the sensors */
}telemetry_init_status;

sensor_init_retval init_functions[] = {
	ASM330LHH_Init,
	BME680_Init,
	BMI323_Init,
	BNO055_Init,
	GPS_Init,
	LIS2MDLTR_Init,
	MS560702BA03_Init
};

const char *sensor_names[] = {
	"ASM330LHH",
	"BME680",
	"BMI323",
	"BNO055",
	"GPS",
	"LIS2MDLTR",
	"MS560702BA03"
};




typedef struct {
    float magnetic_field[3]; // X, Y, Z magnetic field in Gauss
} Magnetometer_3_Axis_Data;

typedef struct {
    float acceleration[3]; // X, Y, Z acceleration
    float gyroscope[3];    // X, Y, Z gyroscope
} IMU_6_Axis_Data;

typedef struct {
    float orientation[3];  // Euler angles: Pitch, Roll, Yaw
    float acceleration[3]; // X, Y, Z acceleration
    float gyroscope[3];    // X, Y, Z gyroscope
    float magnetometer[3]; // X, Y, Z magnetic field
} AHRS_9_Axis_Data;

typedef struct {
    float temperature;  // Temperature in degrees Celsius
    float humidity;     // Relative humidity in %
    float pressure;     // Pressure in hPa
    float gas_resistance; // Gas resistance in Ohms
} Barometer_Data;

typedef struct {
    double latitude;    // Latitude in degrees
    double longitude;   // Longitude in degrees
    float altitude;     // Altitude in meters
    float speed;        // Speed in m/s
    float course;       // Course over ground in degrees
    uint8_t satellites; // Number of satellites in view
} GPS_Data;

typedef struct {
    // Define a structure to hold all sensor data
	IMU_6_Axis_Data asm330lhh_data;
	Barometer_Data bme680_data;
	IMU_6_Axis_Data bmi323_data;
	AHRS_9_Axis_Data bno055_data;
    GPS_Data gps_data;
    IMU_6_Axis_Data lis2mdltr_data;
    Barometer_Data ms560702ba03_data;
} TelemetryData;


telemetry_init_status SensorManager_Init(void);
void SensorManager_UpdateData(TelemetryData *data);


#endif /* INC_TELEMETRY_MANAGER_H_ */
