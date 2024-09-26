/*
 * bno086.h
 *
 *  Created on: Sep 26, 2024
 *      Author: Utilisateur
 */

#ifndef INC_DRIVERS_BNO086_BNO086_H_
#define INC_DRIVERS_BNO086_BNO086_H_

#include "stm32h7xx_hal.h" // Adjust this include based on your STM32 series
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B

// Define the size of the I2C buffer
#define I2C_BUFFER_LENGTH 32

#define STORED_PACKET_SIZE 128
#define SHORT_PACKET_SIZE (STORED_PACKET_SIZE - 4)

// Registers and constants
#define CHANNEL_COMMAND                 0
#define CHANNEL_EXECUTABLE              1
#define CHANNEL_CONTROL                 2
#define CHANNEL_REPORTS                 3
#define CHANNEL_WAKE_REPORTS            4
#define CHANNEL_GYRO                    5

// SHTP Report IDs
#define SHTP_REPORT_COMMAND_RESPONSE    0xF1
#define SHTP_REPORT_COMMAND_REQUEST     0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE   0xF3
#define SHTP_REPORT_FRS_READ_REQUEST    0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9
#define SHTP_REPORT_BASE_TIMESTAMP      0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// Sensor Report IDs
#define SENSOR_REPORTID_ACCELEROMETER                        0x01
#define SENSOR_REPORTID_GYROSCOPE                            0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD                       0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION                  0x04
#define SENSOR_REPORTID_ROTATION_VECTOR                      0x05
#define SENSOR_REPORTID_GRAVITY                              0x06
#define SENSOR_REPORTID_UNCALIBRATED_GYRO                    0x07
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR                 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR          0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR      0x2A
#define SENSOR_REPORTID_TAP_DETECTOR                         0x10
#define SENSOR_REPORTID_STEP_COUNTER                         0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER                 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER                    0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE                        0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER                     0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER         0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR     0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

// Command IDs
#define COMMAND_ERRORS              1
#define COMMAND_COUNTER             2
#define COMMAND_TARE                3
#define COMMAND_INITIALIZE          4
#define COMMAND_DCD                 6
#define COMMAND_ME_CALIBRATE        7
#define COMMAND_DCD_PERIOD_SAVE     9
#define COMMAND_OSCILLATOR          10
#define COMMAND_CLEAR_DCD           11

// Calibration types
#define CALIBRATE_ACCEL             0
#define CALIBRATE_GYRO              1
#define CALIBRATE_MAG               2
#define CALIBRATE_PLANAR_ACCEL      3
#define CALIBRATE_ACCEL_GYRO_MAG    4
#define CALIBRATE_STOP              5

// Tare settings
#define TARE_NOW                    0
#define TARE_PERSIST                1
#define TARE_SET_REORIENTATION      2

#define TARE_AXIS_ALL               0x07
#define TARE_AXIS_Z                 0x04

#define TARE_ROTATION_VECTOR        0
#define TARE_GAME_ROTATION_VECTOR   1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

#define MAX_PACKET_SIZE 128 // Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9  // This is in words. We'll stop at Q point 3

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

// BNO080 device context structure
typedef struct {
    // I2C handle
    I2C_HandleTypeDef *hi2c;

    // I2C device address
    uint8_t deviceAddress;

    // Communication buffers and variables
    uint8_t shtpHeader[4];             // Each packet has a header of 4 bytes
    uint8_t shtpData[MAX_PACKET_SIZE]; // Data buffer
    uint8_t sequenceNumber[6];         // There are 6 com channels. Each channel has its own seqNum
    uint8_t commandSequenceNumber;     // Commands have a seqNum as well
    uint32_t metaData[MAX_METADATA_SIZE]; // Metadata records

    // Sensor values
    uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
    uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
    uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
    uint16_t rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ;
    uint16_t rawBiasGyroX, rawBiasGyroY, rawBiasGyroZ;
    uint16_t UncalibGyroAccuracy;
    uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
    uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
    uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
    uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
    uint8_t tapDetector;
    uint16_t stepCount;
    uint32_t timeStamp;
    uint8_t stabilityClassifier;
    uint8_t activityClassifier;
    uint8_t *activityConfidences; // Array that stores the confidences of the 9 possible activities
    uint8_t calibrationStatus;    // Byte R0 of ME Calibration Response
    uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ;
    uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;
    uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;

    // Q values (fixed-point scaling factors)
    int16_t rotationVector_Q1;
    int16_t rotationVectorAccuracy_Q1;
    int16_t accelerometer_Q1;
    int16_t linear_accelerometer_Q1;
    int16_t gyro_Q1;
    int16_t magnetometer_Q1;
    int16_t angular_velocity_Q1;
    int16_t gravity_Q1;

    // Status flags
    bool hasReset;   // Keeps track of any Reset Complete packets we receive
    bool printDebug; // Flag to enable debugging messages

    // Add any other variables as needed...

} BNO086;

// Function prototypes

// Initialization and configuration
bool BNO080_Begin(uint8_t deviceAddress, uint8_t intPin);
void BNO080_EnableDebugging();
void BNO080_SoftReset();
bool BNO080_HasReset();
uint8_t BNO080_ResetReason();
void BNO080_ModeOn();
void BNO080_ModeSleep();

int8_t BNO086_Init();

// Data conversion
float BNO080_QToFloat(int16_t fixedPointValue, uint8_t qPoint);

// Communication handling
bool BNO080_WaitForI2C();
bool BNO080_ReceivePacket();
bool BNO080_GetData(uint16_t bytesRemaining);
bool BNO080_SendPacket(uint8_t channelNumber, uint8_t dataLength);
void BNO080_PrintPacket();
void BNO080_PrintHeader();

// Sensor enabling functions
void BNO080_EnableRotationVector(uint16_t timeBetweenReports);
void BNO080_EnableGameRotationVector(uint16_t timeBetweenReports);
void BNO080_EnableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
void BNO080_EnableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
void BNO080_EnableAccelerometer(uint16_t timeBetweenReports);
void BNO080_EnableLinearAccelerometer(uint16_t timeBetweenReports);
void BNO080_EnableGravity(uint16_t timeBetweenReports);
void BNO080_EnableGyro(uint16_t timeBetweenReports);
void BNO080_EnableUncalibratedGyro(uint16_t timeBetweenReports);
void BNO080_EnableMagnetometer(uint16_t timeBetweenReports);
void BNO080_EnableTapDetector(uint16_t timeBetweenReports);
void BNO080_EnableStepCounter(uint16_t timeBetweenReports);
void BNO080_EnableStabilityClassifier(uint16_t timeBetweenReports);
void BNO080_EnableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t activityConfidences[9]);
void BNO080_EnableRawAccelerometer(uint16_t timeBetweenReports);
void BNO080_EnableRawGyro(uint16_t timeBetweenReports);
void BNO080_EnableRawMagnetometer(uint16_t timeBetweenReports);
void BNO080_EnableGyroIntegratedRotationVector(uint16_t timeBetweenReports);

// Data availability and parsing
bool BNO080_DataAvailable();
uint16_t BNO080_GetReadings();
uint16_t BNO080_ParseInputReport();
uint16_t BNO080_ParseCommandReport();

// Sensor data retrieval
void BNO080_GetQuat(float *i, float *j, float *k, float *real, float *radAccuracy, uint8_t *accuracy);
float BNO080_GetQuatI();
float BNO080_GetQuatJ();
float BNO080_GetQuatK();
float BNO080_GetQuatReal();
float BNO080_GetQuatRadianAccuracy();
uint8_t BNO080_GetQuatAccuracy();

void BNO080_GetAccel(float *x, float *y, float *z, uint8_t *accuracy);
float BNO080_GetAccelX();
float BNO080_GetAccelY();
float BNO080_GetAccelZ();
uint8_t BNO080_GetAccelAccuracy();

void BNO080_GetLinAccel(float *x, float *y, float *z, uint8_t *accuracy);
float BNO080_GetLinAccelX();
float BNO080_GetLinAccelY();
float BNO080_GetLinAccelZ();
uint8_t BNO080_GetLinAccelAccuracy();

void BNO080_GetGyro(float *x, float *y, float *z, uint8_t *accuracy);
float BNO080_GetGyroX();
float BNO080_GetGyroY();
float BNO080_GetGyroZ();
uint8_t BNO080_GetGyroAccuracy();

void BNO080_GetUncalibratedGyro(float *x, float *y, float *z, float *bx, float *by, float *bz, uint8_t *accuracy);
float BNO080_GetUncalibratedGyroX();
float BNO080_GetUncalibratedGyroY();
float BNO080_GetUncalibratedGyroZ();
float BNO080_GetUncalibratedGyroBiasX();
float BNO080_GetUncalibratedGyroBiasY();
float BNO080_GetUncalibratedGyroBiasZ();
uint8_t BNO080_GetUncalibratedGyroAccuracy();

void BNO080_GetMag(float *x, float *y, float *z, uint8_t *accuracy);
float BNO080_GetMagX();
float BNO080_GetMagY();
float BNO080_GetMagZ();
uint8_t BNO080_GetMagAccuracy();

void BNO080_GetGravity(float *x, float *y, float *z, uint8_t *accuracy);
float BNO080_GetGravityX();
float BNO080_GetGravityY();
float BNO080_GetGravityZ();
uint8_t BNO080_GetGravityAccuracy();

// Calibration functions
void BNO080_CalibrateAccelerometer();
void BNO080_CalibrateGyro();
void BNO080_CalibrateMagnetometer();
void BNO080_CalibratePlanarAccelerometer();
void BNO080_CalibrateAll();
void BNO080_EndCalibration();
void BNO080_SaveCalibration();
void BNO080_RequestCalibrationStatus();
bool BNO080_CalibrationComplete();

// Tare functions
void BNO080_TareNow(bool zAxis, uint8_t rotationVectorBasis);
void BNO080_SaveTare();
void BNO080_ClearTare();

// Additional data retrieval functions
uint8_t BNO080_GetTapDetector();
uint32_t BNO080_GetTimeStamp();
uint16_t BNO080_GetStepCount();
uint8_t BNO080_GetStabilityClassifier();
uint8_t BNO080_GetActivityClassifier();

int16_t BNO080_GetRawAccelX();
int16_t BNO080_GetRawAccelY();
int16_t BNO080_GetRawAccelZ();

int16_t BNO080_GetRawGyroX();
int16_t BNO080_GetRawGyroY();
int16_t BNO080_GetRawGyroZ();

int16_t BNO080_GetRawMagX();
int16_t BNO080_GetRawMagY();
int16_t BNO080_GetRawMagZ();

float BNO080_GetRoll();
float BNO080_GetPitch();
float BNO080_GetYaw();

// Command functions
void BNO080_SetFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
void BNO080_SetFeatureCommand_Specific(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
void BNO080_SendCommand(uint8_t command);
void BNO080_SendCalibrateCommand(uint8_t thingToCalibrate);
void BNO080_SendTareCommand(uint8_t command, uint8_t axis, uint8_t rotationVectorBasis);

// Metadata functions
int16_t BNO080_GetQ1(uint16_t recordID);
int16_t BNO080_GetQ2(uint16_t recordID);
int16_t BNO080_GetQ3(uint16_t recordID);
float BNO080_GetResolution(uint16_t recordID);
float BNO080_GetRange(uint16_t recordID);
uint32_t BNO080_ReadFRSword(uint16_t recordID, uint8_t wordNumber);
void BNO080_FRSReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
bool BNO080_ReadFRSData(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

#endif /* INC_DRIVERS_BNO086_BNO086_H_ */
