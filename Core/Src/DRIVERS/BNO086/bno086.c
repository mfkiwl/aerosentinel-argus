/*
 * bno086.c
 *
 *  Created on: Sep 26, 2024
 *      Author: Utilisateur
 */

#include "DRIVERS/BNO086/bno086.h"
#include <stdio.h>

// Assume hi2c1 is defined and initialized elsewhere in your project
extern I2C_HandleTypeDef hi2c1;

static uint8_t shtpData[STORED_PACKET_SIZE];
static uint16_t packetLength = 0;

// Global device context
BNO086 bno086_dev;

void waitForDeviceRdy(){
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) != GPIO_PIN_RESET){
		;
	}
}

// Initialization and configuration

bool BNO080_Begin(uint8_t deviceAddress, uint8_t intPin)
{
    // Wait for device ready
    waitForDeviceRdy();

    // Store the I2C address (STM32 HAL uses 8-bit addresses)
    bno086_dev.deviceAddress = deviceAddress << 1;

    // Assign the I2C handle
    bno086_dev.hi2c = &hi2c1;

    // Initialize sequence numbers
    memset(bno086_dev.sequenceNumber, 0, sizeof(bno086_dev.sequenceNumber));
    bno086_dev.commandSequenceNumber = 0;

    // Initialize Q values (fixed-point scaling factors)
    bno086_dev.rotationVector_Q1 = 14;
    bno086_dev.rotationVectorAccuracy_Q1 = 12;
    bno086_dev.accelerometer_Q1 = 8;
    bno086_dev.linear_accelerometer_Q1 = 8;
    bno086_dev.gyro_Q1 = 9;
    bno086_dev.magnetometer_Q1 = 4;
    bno086_dev.angular_velocity_Q1 = 10;
    bno086_dev.gravity_Q1 = 8;

    // Initialize status flags
    bno086_dev.hasReset = false;
    bno086_dev.printDebug = false;

    // Reset the sensor (optional, you can uncomment if needed)
    // BNO080_SoftReset();

    // Check communication with the device
    bno086_dev.shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID
    bno086_dev.shtpData[1] = 0; // Reserved

    // Transmit packet on CHANNEL_CONTROL (channel 2), 2 bytes
    if (!BNO080_SendPacket(CHANNEL_CONTROL, 2))
    {
        return false;
    }

    // Wait for response
    waitForDeviceRdy();
    if (BNO080_ReceivePacket())
    {
        if (bno086_dev.shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        {
            // Optionally process product ID information
            return true;
        }
    }

    return false; // Communication failed
}

void BNO080_EnableDebugging()
{
    bno086_dev.printDebug = true;
}

void BNO080_SoftReset()
{
    bno086_dev.shtpData[0] = 1; // Reset command

    // Send reset command on CHANNEL_EXECUTABLE (channel 1)
    BNO080_SendPacket(CHANNEL_EXECUTABLE, 1);

    // Flush incoming data
    HAL_Delay(50);
    while (BNO080_ReceivePacket())
        ;
    HAL_Delay(50);
    while (BNO080_ReceivePacket())
        ;
}

bool BNO080_HasReset()
{
    if (bno086_dev.hasReset)
    {
        bno086_dev.hasReset = false;
        return true;
    }
    return false;
}

uint8_t BNO080_ResetReason()
{
    bno086_dev.shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID
    bno086_dev.shtpData[1] = 0; // Reserved

    // Send packet on CHANNEL_CONTROL
    if (!BNO080_SendPacket(CHANNEL_CONTROL, 2))
    {
        return 0;
    }

    // Wait for response
    if (BNO080_ReceivePacket())
    {
        if (bno086_dev.shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        {
            return bno086_dev.shtpData[1]; // Reset reason code
        }
    }

    return 0; // Failed to get reset reason
}

void BNO080_ModeOn()
{
    bno086_dev.shtpData[0] = 2; // On mode

    // Send command on CHANNEL_EXECUTABLE
    BNO080_SendPacket(CHANNEL_EXECUTABLE, 1);

    // Flush incoming data
    HAL_Delay(50);
    while (BNO080_ReceivePacket())
        ;
    HAL_Delay(50);
    while (BNO080_ReceivePacket())
        ;
}

void BNO080_ModeSleep()
{
    bno086_dev.shtpData[0] = 3; // Sleep mode

    // Send command on CHANNEL_EXECUTABLE
    BNO080_SendPacket(CHANNEL_EXECUTABLE, 1);

    // Flush incoming data
    HAL_Delay(50);
    while (BNO080_ReceivePacket())
        ;
    HAL_Delay(50);
    while (BNO080_ReceivePacket())
        ;
}

// Data conversion

float BNO080_QToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    return ((float)fixedPointValue) / ((float)(1 << qPoint));
}

// Communication handling

bool BNO080_WaitForI2C()
{
    // In STM32 HAL, functions are blocking, so this is typically not needed
    // If you have a non-blocking implementation, you can handle timeouts here
    return true;
}

// Modify BNO080_ReceivePacket to wait for data ready
bool BNO080_ReceivePacket(void)
{
    uint8_t header[4];
    uint8_t ret;
    uint16_t packetLength;

    // Wait for data ready signal
    waitForDeviceRdy();

    // Read the 4-byte header
    ret = HAL_I2C_Master_Receive(bno086_dev.hi2c, bno086_dev.deviceAddress, header, 4, HAL_MAX_DELAY);
    if (ret != HAL_OK)
    {
        return false;
    }

    // Store the header
    bno086_dev.shtpHeader[0] = header[0];
    bno086_dev.shtpHeader[1] = header[1];
    bno086_dev.shtpHeader[2] = header[2];
    bno086_dev.shtpHeader[3] = header[3];

    // Calculate the number of data bytes in this packet
    packetLength = ((uint16_t)header[1] << 8) | header[0];
    packetLength &= ~(1 << 15); // Clear the MSbit (continuation bit)

    if (packetLength == 0)
    {
        // Packet is empty
        return false;
    }

    packetLength -= 4; // Remove the header bytes from the data count

    // Read the remaining data into shtpData
    if (!BNO080_GetData(packetLength))
    {
        return false;
    }

    // Check for reset complete packet
    if (bno086_dev.shtpHeader[2] == CHANNEL_EXECUTABLE && bno086_dev.shtpData[0] == EXECUTABLE_RESET_COMPLETE)
    {
        bno086_dev.hasReset = true;
    }

    return true;
}


bool BNO080_GetData(uint16_t bytesRemaining)
{
    uint16_t dataSpot = 0; // Start at the beginning of shtpData array
    uint8_t ret;

    while (bytesRemaining > 0)
    {
        uint16_t numberOfBytesToRead = bytesRemaining;

        // Adjust for I2C buffer limitations if necessary
        uint16_t maxI2CBufferSize = 28; // Adjust based on your MCU's I2C buffer size

        if (numberOfBytesToRead > (maxI2CBufferSize - 4))
            numberOfBytesToRead = (maxI2CBufferSize - 4);

        // Read numberOfBytesToRead + 4 bytes (header + data)
        uint8_t buffer[32]; // Adjust size if needed

        ret = HAL_I2C_Master_Receive(bno086_dev.hi2c, bno086_dev.deviceAddress, buffer, numberOfBytesToRead + 4, HAL_MAX_DELAY);
        if (ret != HAL_OK)
        {
            return false;
        }

        // Skip the first 4 bytes (header)
        for (uint16_t i = 0; i < numberOfBytesToRead; i++)
        {
            if (dataSpot < MAX_PACKET_SIZE)
            {
                bno086_dev.shtpData[dataSpot++] = buffer[i + 4]; // Skip header
            }
        }

        bytesRemaining -= numberOfBytesToRead;
    }

    return true;
}



bool BNO080_SendPacket(uint8_t channelNumber, uint8_t dataLength)
{
    uint8_t packet[MAX_PACKET_SIZE];
    uint8_t ret;
    uint16_t len = dataLength + 4; // Add 4 bytes for the header

    if (len > MAX_PACKET_SIZE)
    {
        // Packet too large
        return false;
    }

    // Build the packet header
    packet[0] = (len & 0xFF);               // Packet length LSB
    packet[1] = (len >> 8) & 0xFF;          // Packet length MSB
    packet[2] = channelNumber;              // Channel number
    packet[3] = bno086_dev.sequenceNumber[channelNumber]++; // Sequence number

    // Copy the data into the packet
    memcpy(&packet[4], bno086_dev.shtpData, dataLength);

    // Send the packet over I2C
    ret = HAL_I2C_Master_Transmit(bno086_dev.hi2c, bno086_dev.deviceAddress, packet, len, HAL_MAX_DELAY);
    if (ret != HAL_OK)
    {
        // Transmission failed
        return false;
    }

    return true;
}



void BNO080_PrintPacket()
{
    if (bno086_dev.printDebug)
    {
        uint16_t packetLength = ((uint16_t)bno086_dev.shtpHeader[1] << 8) | bno086_dev.shtpHeader[0];
        packetLength &= ~(1 << 15); // Clear continuation bit

        // Print the header
        printf("Header: ");
        for (int i = 0; i < 4; i++)
        {
            printf("%02X ", bno086_dev.shtpHeader[i]);
        }

        // Print the data
        printf("Data: ");
        for (int i = 0; i < packetLength - 4; i++)
        {
            printf("%02X ", bno086_dev.shtpData[i]);
        }
        printf("\n");
    }
}

void BNO080_PrintHeader()
{
    if (bno086_dev.printDebug)
    {
        printf("Header: ");
        for (int i = 0; i < 4; i++)
        {
            printf("%02X ", bno086_dev.shtpHeader[i]);
        }
        printf("\n");
    }
}

// Sensor enabling functions

void BNO080_EnableRotationVector(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}

void BNO080_EnableGameRotationVector(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

void BNO080_EnableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

void BNO080_EnableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

void BNO080_EnableAccelerometer(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

void BNO080_EnableLinearAccelerometer(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

void BNO080_EnableGravity(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_GRAVITY, timeBetweenReports);
}

void BNO080_EnableGyro(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

void BNO080_EnableUncalibratedGyro(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_UNCALIBRATED_GYRO, timeBetweenReports);
}

void BNO080_EnableMagnetometer(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

void BNO080_EnableTapDetector(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports);
}

void BNO080_EnableStepCounter(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

void BNO080_EnableStabilityClassifier(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

void BNO080_EnableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t activityConfidences[9])
{
    bno086_dev.activityConfidences = activityConfidences;

    BNO080_SetFeatureCommand_Specific(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

void BNO080_EnableRawAccelerometer(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

void BNO080_EnableRawGyro(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

void BNO080_EnableRawMagnetometer(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

void BNO080_EnableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);
}

// Data availability and parsing

bool BNO080_DataAvailable()
{
    return (BNO080_GetReadings() != 0);
}

uint16_t BNO080_GetReadings()
{
    if (BNO080_ReceivePacket())
    {
        // Check the channel
        if (bno086_dev.shtpHeader[2] == CHANNEL_REPORTS && bno086_dev.shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
        {
            return BNO080_ParseInputReport();
        }
        else if (bno086_dev.shtpHeader[2] == CHANNEL_CONTROL)
        {
            return BNO080_ParseCommandReport();
        }
        else if (bno086_dev.shtpHeader[2] == CHANNEL_GYRO)
        {
            return BNO080_ParseInputReport();
        }
    }
    return 0;
}

uint16_t BNO080_ParseInputReport()
{
    // Calculate data length
    uint16_t dataLength = ((uint16_t)bno086_dev.shtpHeader[1] << 8) | bno086_dev.shtpHeader[0];
    dataLength &= ~(1 << 15); // Clear continuation bit
    dataLength -= 4; // Subtract header length

    // Timestamp
    bno086_dev.timeStamp = ((uint32_t)bno086_dev.shtpData[4] << 24) | ((uint32_t)bno086_dev.shtpData[3] << 16) |
                    ((uint32_t)bno086_dev.shtpData[2] << 8) | bno086_dev.shtpData[1];

    // Handle gyro-integrated reports separately
    if (bno086_dev.shtpHeader[2] == CHANNEL_GYRO)
    {
        bno086_dev.rawQuatI = (uint16_t)(bno086_dev.shtpData[1] << 8 | bno086_dev.shtpData[0]);
        bno086_dev.rawQuatJ = (uint16_t)(bno086_dev.shtpData[3] << 8 | bno086_dev.shtpData[2]);
        bno086_dev.rawQuatK = (uint16_t)(bno086_dev.shtpData[5] << 8 | bno086_dev.shtpData[4]);
        bno086_dev.rawQuatReal = (uint16_t)(bno086_dev.shtpData[7] << 8 | bno086_dev.shtpData[6]);
        bno086_dev.rawFastGyroX = (uint16_t)(bno086_dev.shtpData[9] << 8 | bno086_dev.shtpData[8]);
        bno086_dev.rawFastGyroY = (uint16_t)(bno086_dev.shtpData[11] << 8 | bno086_dev.shtpData[10]);
        bno086_dev.rawFastGyroZ = (uint16_t)(bno086_dev.shtpData[13] << 8 | bno086_dev.shtpData[12]);

        return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
    }

    uint8_t reportID = bno086_dev.shtpData[5]; // Report ID
    uint8_t status = bno086_dev.shtpData[7] & 0x03; // Status bits

    uint16_t data1 = (uint16_t)(bno086_dev.shtpData[10] << 8 | bno086_dev.shtpData[9]);
    uint16_t data2 = (uint16_t)(bno086_dev.shtpData[12] << 8 | bno086_dev.shtpData[11]);
    uint16_t data3 = (uint16_t)(bno086_dev.shtpData[14] << 8 | bno086_dev.shtpData[13]);
    uint16_t data4 = (uint16_t)(bno086_dev.shtpData[16] << 8 | bno086_dev.shtpData[15]);
    uint16_t data5 = (uint16_t)(bno086_dev.shtpData[18] << 8 | bno086_dev.shtpData[17]);
    uint16_t data6 = (uint16_t)(bno086_dev.shtpData[20] << 8 | bno086_dev.shtpData[19]);

    // Extract data based on report ID
    switch (reportID)
    {
    case SENSOR_REPORTID_ACCELEROMETER:
        bno086_dev.accelAccuracy = status;
        bno086_dev.rawAccelX = data1;
        bno086_dev.rawAccelY = data2;
        bno086_dev.rawAccelZ = data3;
        break;

    case SENSOR_REPORTID_LINEAR_ACCELERATION:
        bno086_dev.accelLinAccuracy = status;
        bno086_dev.rawLinAccelX = data1;
        bno086_dev.rawLinAccelY = data2;
        bno086_dev.rawLinAccelZ = data3;
        break;

    case SENSOR_REPORTID_GYROSCOPE:
        bno086_dev.gyroAccuracy = status;
        bno086_dev.rawGyroX = data1;
        bno086_dev.rawGyroY = data2;
        bno086_dev.rawGyroZ = data3;
        break;

    case SENSOR_REPORTID_UNCALIBRATED_GYRO:
        bno086_dev.UncalibGyroAccuracy = status;
        bno086_dev.rawUncalibGyroX = data1;
        bno086_dev.rawUncalibGyroY = data2;
        bno086_dev.rawUncalibGyroZ = data3;
        bno086_dev.rawBiasGyroX = data4;
        bno086_dev.rawBiasGyroY = data5;
        bno086_dev.rawBiasGyroZ = data6;
        break;

    case SENSOR_REPORTID_MAGNETIC_FIELD:
        bno086_dev.magAccuracy = status;
        bno086_dev.rawMagX = data1;
        bno086_dev.rawMagY = data2;
        bno086_dev.rawMagZ = data3;
        break;

    case SENSOR_REPORTID_ROTATION_VECTOR:
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
    case SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR:
    case SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR:
        bno086_dev.quatAccuracy = status;
        bno086_dev.rawQuatI = data1;
        bno086_dev.rawQuatJ = data2;
        bno086_dev.rawQuatK = data3;
        bno086_dev.rawQuatReal = data4;
        bno086_dev.rawQuatRadianAccuracy = data5;
        break;

    case SENSOR_REPORTID_TAP_DETECTOR:
        bno086_dev.tapDetector = bno086_dev.shtpData[9]; // Byte 4
        break;

    case SENSOR_REPORTID_STEP_COUNTER:
        bno086_dev.stepCount = data3; // Bytes 8/9
        break;

    case SENSOR_REPORTID_STABILITY_CLASSIFIER:
        bno086_dev.stabilityClassifier = bno086_dev.shtpData[9]; // Byte 4
        break;

    case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
        bno086_dev.activityClassifier = bno086_dev.shtpData[10]; // Most likely state
        // Copy confidences
        for (uint8_t i = 0; i < 9; i++)
        {
            bno086_dev.activityConfidences[i] = bno086_dev.shtpData[11 + i];
        }
        break;

    case SENSOR_REPORTID_RAW_ACCELEROMETER:
        bno086_dev.memsRawAccelX = data1;
        bno086_dev.memsRawAccelY = data2;
        bno086_dev.memsRawAccelZ = data3;
        break;

    case SENSOR_REPORTID_RAW_GYROSCOPE:
        bno086_dev.memsRawGyroX = data1;
        bno086_dev.memsRawGyroY = data2;
        bno086_dev.memsRawGyroZ = data3;
        break;

    case SENSOR_REPORTID_RAW_MAGNETOMETER:
        bno086_dev.memsRawMagX = data1;
        bno086_dev.memsRawMagY = data2;
        bno086_dev.memsRawMagZ = data3;
        break;

    case SENSOR_REPORTID_GRAVITY:
        bno086_dev.gravityAccuracy = status;
        bno086_dev.gravityX = data1;
        bno086_dev.gravityY = data2;
        bno086_dev.gravityZ = data3;
        break;

    default:
        // Unknown report ID
        break;
    }

    return reportID;
}

uint16_t BNO080_ParseCommandReport()
{
    if (bno086_dev.shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        uint8_t command = bno086_dev.shtpData[2]; // Command byte
        if (command == COMMAND_ME_CALIBRATE)
        {
            bno086_dev.calibrationStatus = bno086_dev.shtpData[5]; // Status byte
        }
        return bno086_dev.shtpData[0];
    }
    return 0;
}

// Sensor data retrieval

void BNO080_GetQuat(float *i, float *j, float *k, float *real, float *radAccuracy, uint8_t *accuracy)
{
    *i = BNO080_QToFloat((int16_t)bno086_dev.rawQuatI, bno086_dev.rotationVector_Q1);
    *j = BNO080_QToFloat((int16_t)bno086_dev.rawQuatJ, bno086_dev.rotationVector_Q1);
    *k = BNO080_QToFloat((int16_t)bno086_dev.rawQuatK, bno086_dev.rotationVector_Q1);
    *real = BNO080_QToFloat((int16_t)bno086_dev.rawQuatReal, bno086_dev.rotationVector_Q1);
    *radAccuracy = BNO080_QToFloat((int16_t)bno086_dev.rawQuatRadianAccuracy, bno086_dev.rotationVectorAccuracy_Q1);
    *accuracy = bno086_dev.quatAccuracy;
}

float BNO080_GetQuatI()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawQuatI, bno086_dev.rotationVector_Q1);
}

float BNO080_GetQuatJ()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawQuatJ, bno086_dev.rotationVector_Q1);
}

float BNO080_GetQuatK()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawQuatK, bno086_dev.rotationVector_Q1);
}

float BNO080_GetQuatReal()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawQuatReal, bno086_dev.rotationVector_Q1);
}

float BNO080_GetQuatRadianAccuracy()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawQuatRadianAccuracy, bno086_dev.rotationVectorAccuracy_Q1);
}

uint8_t BNO080_GetQuatAccuracy()
{
    return bno086_dev.quatAccuracy;
}

void BNO080_GetAccel(float *x, float *y, float *z, uint8_t *accuracy)
{
    *x = BNO080_QToFloat((int16_t)bno086_dev.rawAccelX, bno086_dev.accelerometer_Q1);
    *y = BNO080_QToFloat((int16_t)bno086_dev.rawAccelY, bno086_dev.accelerometer_Q1);
    *z = BNO080_QToFloat((int16_t)bno086_dev.rawAccelZ, bno086_dev.accelerometer_Q1);
    *accuracy = bno086_dev.accelAccuracy;
}

float BNO080_GetAccelX()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawAccelX, bno086_dev.accelerometer_Q1);
}

float BNO080_GetAccelY()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawAccelY, bno086_dev.accelerometer_Q1);
}

float BNO080_GetAccelZ()
{
    return BNO080_QToFloat((int16_t)bno086_dev.rawAccelZ, bno086_dev.accelerometer_Q1);
}

uint8_t BNO080_GetAccelAccuracy()
{
    return bno086_dev.accelAccuracy;
}

// Similar implementations for other sensor data retrieval functions...

// Calibration functions

void BNO080_CalibrateAccelerometer()
{
    BNO080_SendCalibrateCommand(CALIBRATE_ACCEL);
}

void BNO080_CalibrateGyro()
{
    BNO080_SendCalibrateCommand(CALIBRATE_GYRO);
}

void BNO080_CalibrateMagnetometer()
{
    BNO080_SendCalibrateCommand(CALIBRATE_MAG);
}

void BNO080_CalibratePlanarAccelerometer()
{
    BNO080_SendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

void BNO080_CalibrateAll()
{
    BNO080_SendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO080_EndCalibration()
{
    BNO080_SendCalibrateCommand(CALIBRATE_STOP);
}

void BNO080_SaveCalibration()
{
    memset(&bno086_dev.shtpData[3], 0, 9); // Clear parameters P0-P8

    // Send DCD save command
    BNO080_SendCommand(COMMAND_DCD);
}

void BNO080_RequestCalibrationStatus()
{
    memset(&bno086_dev.shtpData[3], 0, 9); // Clear parameters P0-P8
    bno086_dev.shtpData[6] = 0x01; // Subcommand: Get ME Calibration

    BNO080_SendCommand(COMMAND_ME_CALIBRATE);
}

bool BNO080_CalibrationComplete()
{
    return (bno086_dev.calibrationStatus == 0);
}

// Tare functions

void BNO080_TareNow(bool zAxis, uint8_t rotationVectorBasis)
{
    memset(&bno086_dev.shtpData[3], 0, 9); // Clear parameters P0-P8

    bno086_dev.shtpData[3] = TARE_NOW;
    bno086_dev.shtpData[4] = zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL;
    bno086_dev.shtpData[5] = rotationVectorBasis;

    BNO080_SendCommand(COMMAND_TARE);
}

void BNO080_SaveTare()
{
    memset(&bno086_dev.shtpData[3], 0, 9); // Clear parameters P0-P8

    bno086_dev.shtpData[3] = TARE_PERSIST;

    BNO080_SendCommand(COMMAND_TARE);
}

void BNO080_ClearTare()
{
    memset(&bno086_dev.shtpData[3], 0, 9); // Clear parameters P0-P8

    bno086_dev.shtpData[3] = TARE_SET_REORIENTATION;

    BNO080_SendCommand(COMMAND_TARE);
}

// Additional data retrieval functions

uint8_t BNO080_GetTapDetector()
{
    uint8_t tap = bno086_dev.tapDetector;
    bno086_dev.tapDetector = 0; // Reset after reading
    return tap;
}

uint32_t BNO080_GetTimeStamp()
{
    return bno086_dev.timeStamp;
}

uint16_t BNO080_GetStepCount()
{
    return bno086_dev.stepCount;
}

uint8_t BNO080_GetStabilityClassifier()
{
    return bno086_dev.stabilityClassifier;
}

uint8_t BNO080_GetActivityClassifier()
{
    return bno086_dev.activityClassifier;
}

int16_t BNO080_GetRawAccelX()
{
    return bno086_dev.memsRawAccelX;
}

int16_t BNO080_GetRawAccelY()
{
    return bno086_dev.memsRawAccelY;
}

int16_t BNO080_GetRawAccelZ()
{
    return bno086_dev.memsRawAccelZ;
}

int16_t BNO080_GetRawGyroX()
{
    return bno086_dev.memsRawGyroX;
}

int16_t BNO080_GetRawGyroY()
{
    return bno086_dev.memsRawGyroY;
}

int16_t BNO080_GetRawGyroZ()
{
    return bno086_dev.memsRawGyroZ;
}

int16_t BNO080_GetRawMagX()
{
    return bno086_dev.memsRawMagX;
}

int16_t BNO080_GetRawMagY()
{
    return bno086_dev.memsRawMagY;
}

int16_t BNO080_GetRawMagZ()
{
    return bno086_dev.memsRawMagZ;
}

float BNO080_GetRoll()
{
    float qw = BNO080_GetQuatReal();
    float qx = BNO080_GetQuatI();
    float qy = BNO080_GetQuatJ();
    float qz = BNO080_GetQuatK();

    float ysqr = qy * qy;

    float t0 = +2.0f * (qw * qx + qy * qz);
    float t1 = +1.0f - 2.0f * (qx * qx + ysqr);
    return atan2f(t0, t1);
}

float BNO080_GetPitch()
{
    float qw = BNO080_GetQuatReal();
    float qx = BNO080_GetQuatI();
    float qy = BNO080_GetQuatJ();
    float qz = BNO080_GetQuatK();

    float t2 = +2.0f * (qw * qy - qz * qx);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    return asinf(t2);
}

float BNO080_GetYaw()
{
    float qw = BNO080_GetQuatReal();
    float qx = BNO080_GetQuatI();
    float qy = BNO080_GetQuatJ();
    float qz = BNO080_GetQuatK();

    float ysqr = qy * qy;

    float t3 = +2.0f * (qw * qz + qx * qy);
    float t4 = +1.0f - 2.0f * (ysqr + qz * qz);
    return atan2f(t3, t4);
}

// Command functions

void BNO080_SetFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
    BNO080_SetFeatureCommand_Specific(reportID, timeBetweenReports, 0);
}

void BNO080_SetFeatureCommand_Specific(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
    uint32_t interval_us = timeBetweenReports * 1000; // Convert ms to us

    bno086_dev.shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND; // Report ID
    bno086_dev.shtpData[1] = reportID;                        // Feature Report ID
    bno086_dev.shtpData[2] = 0;                               // Feature flags
    bno086_dev.shtpData[3] = 0;                               // Change sensitivity LSB
    bno086_dev.shtpData[4] = 0;                               // Change sensitivity MSB
    bno086_dev.shtpData[5] = (interval_us >> 0) & 0xFF;       // Report interval LSB
    bno086_dev.shtpData[6] = (interval_us >> 8) & 0xFF;
    bno086_dev.shtpData[7] = (interval_us >> 16) & 0xFF;
    bno086_dev.shtpData[8] = (interval_us >> 24) & 0xFF;      // Report interval MSB
    bno086_dev.shtpData[9] = 0;                               // Batch Interval LSB
    bno086_dev.shtpData[10] = 0;
    bno086_dev.shtpData[11] = 0;
    bno086_dev.shtpData[12] = 0;                              // Batch Interval MSB
    bno086_dev.shtpData[13] = (specificConfig >> 0) & 0xFF;   // Sensor-specific config LSB
    bno086_dev.shtpData[14] = (specificConfig >> 8) & 0xFF;
    bno086_dev.shtpData[15] = (specificConfig >> 16) & 0xFF;
    bno086_dev.shtpData[16] = (specificConfig >> 24) & 0xFF;  // Sensor-specific config MSB

    // Send the packet
    BNO080_SendPacket(CHANNEL_CONTROL, 17);
}

void BNO080_SendCommand(uint8_t command)
{
    bno086_dev.shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; // Command Request
    bno086_dev.shtpData[1] = bno086_dev.commandSequenceNumber++; // Sequence number
    bno086_dev.shtpData[2] = command;                     // Command

    // Clear parameters P0-P8
    memset(&bno086_dev.shtpData[3], 0, 9);

    // Send the packet
    BNO080_SendPacket(CHANNEL_CONTROL, 12);
}

void BNO080_SendCalibrateCommand(uint8_t thingToCalibrate)
{
    memset(&bno086_dev.shtpData[3], 0, 9); // Clear parameters P0-P8

    switch (thingToCalibrate)
    {
    case CALIBRATE_ACCEL:
        bno086_dev.shtpData[3] = 1;
        break;
    case CALIBRATE_GYRO:
        bno086_dev.shtpData[4] = 1;
        break;
    case CALIBRATE_MAG:
        bno086_dev.shtpData[5] = 1;
        break;
    case CALIBRATE_PLANAR_ACCEL:
        bno086_dev.shtpData[7] = 1;
        break;
    case CALIBRATE_ACCEL_GYRO_MAG:
        bno086_dev.shtpData[3] = 1;
        bno086_dev.shtpData[4] = 1;
        bno086_dev.shtpData[5] = 1;
        break;
    case CALIBRATE_STOP:
        // All parameters are zero
        break;
    default:
        break;
    }

    bno086_dev.calibrationStatus = 1; // Non-zero means calibration in progress

    BNO080_SendCommand(COMMAND_ME_CALIBRATE);
}

void BNO080_SendTareCommand(uint8_t command, uint8_t axis, uint8_t rotationVectorBasis)
{
    memset(&bno086_dev.shtpData[3], 0, 9); // Clear parameters P0-P8

    bno086_dev.shtpData[3] = command;

    if (command == TARE_NOW)
    {
        bno086_dev.shtpData[4] = axis; // Axis setting
        bno086_dev.shtpData[5] = rotationVectorBasis; // Rotation vector
    }

    BNO080_SendCommand(COMMAND_TARE);
}

// Metadata functions

// Implement the metadata functions as needed...


int8_t BNO086_Init(){
	int8_t ret;
	ret = BNO080_Begin(BNO080_DEFAULT_ADDRESS, 255);
	if (ret != true) {
		return 1;
	}

	// Enable Rotation Vector reports every 100ms
	    BNO080_EnableRotationVector(100);

	return 0;
}
