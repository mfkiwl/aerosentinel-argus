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
#include "DRIVERS/ADXL375/ADXL375.h"
#include "DRIVERS/MPL311/mpl311.h"
#include "DRIVERS/BNO086/bno086.h"

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
	AHRS_9_Axis_Data bno055_data;
    Barometer_2_Axis ms5607_data;
	HigG_Accemerometer_3_Axis adxl375_data;
	Altimeter_3_Axis mpl311_data;
} TelemetryData;


telemetry_init_status SensorManager_Init(void);
void SensorManager_UpdateData(TelemetryData *data);
void TestTelemetry();


#endif /* INC_DATA_MANAGEMENT_TELEMETRY_MANAGER_H_ */
