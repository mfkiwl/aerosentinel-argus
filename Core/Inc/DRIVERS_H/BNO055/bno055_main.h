/*
 * bno055_main.h
 *
 *  Created on: Jul 14, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_BNO055_BNO055_MAIN_H_
#define INC_DRIVERS_H_BNO055_BNO055_MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx.h"
#include <stdbool.h>
#include "DRIVERS_H/BNO055/bno055.h"

#define BUS_TIMEOUT             1000
#define QUAT_SCALING_FACT		16384.0

typedef struct{
	double w;
	double x;
	double y;
	double z;
} quaternion_vect;

typedef struct {
    float orientation[3];  // Euler angles: Pitch, Roll, Yaw
    float orientation_q[3];  // Euler angles from quaternion conversion : Pitch, Roll, Yaw
    double acceleration[3]; // X, Y, Z acceleration
    double gyroscope[3];    // X, Y, Z gyroscope
    float gravity[3]; // X, Y, Z gravity
    double magnetometer[3]; // X, Y, Z magnetic field
    float temperature;
    quaternion_vect quaternion; 	// W, X, Y, Z quaternion vectors
} AHRS_9_Axis_Data;

typedef struct {
    // Accelerometer data
    double accel_x_ms2;
    double accel_x_mg;
    double accel_y_ms2;
    double accel_y_mg;
    double accel_z_ms2;
    double accel_z_mg;
    struct bno055_accel_double_t accel_xyz_ms2;
    struct bno055_accel_double_t accel_xyz_mg;

    // Magnetometer data
    double mag_x_uT;
    double mag_y_uT;
    double mag_z_uT;
    struct bno055_mag_double_t mag_xyz_uT;

    // Gyroscope data
    double gyro_x_dps;
    double gyro_y_dps;
    double gyro_z_dps;
    double gyro_x_rps;
    double gyro_y_rps;
    double gyro_z_rps;
    struct bno055_gyro_double_t gyro_xyz_dps;
    struct bno055_gyro_double_t gyro_xyz_rps;

    // Euler angles data
    double euler_h_deg;
    double euler_r_deg;
    double euler_p_deg;
    double euler_h_rad;
    double euler_r_rad;
    double euler_p_rad;
    struct bno055_euler_double_t euler_hpr_deg;
    struct bno055_euler_double_t euler_hpr_rad;

    // Linear acceleration data
    double linear_accel_x_ms2;
    double linear_accel_y_ms2;
    double linear_accel_z_ms2;
    struct bno055_linear_accel_double_t linear_accel_xyz_ms2;

    // Gravity data
    double gravity_x_ms2;
    double gravity_y_ms2;
    double gravity_z_ms2;
    struct bno055_gravity_double_t gravity_xyz_ms2;
} AHRS_Converted_Data;

signed char bno055_platform_read(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char ReadNumbr);
signed char bno055_platform_write(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char WriteNumbr);

void bno055_delay_msec(u32 msecs);

int8_t BNO055_Init();

AHRS_9_Axis_Data bno_read_fusion_data();
void bno055_print_fusion_data(AHRS_9_Axis_Data *data);

AHRS_Converted_Data bno_read_converted_data();
void print_bno055_data(AHRS_Converted_Data *data);

#endif /* INC_DRIVERS_H_BNO055_BNO055_MAIN_H_ */
