#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

//##################################################################################################################

typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;
	
	float				Latitude;
	double			LatitudeDecimal;
	char				NS_Indicator;
	float				Longitude;
	double			LongitudeDecimal;
	char				EW_Indicator;
	
	uint8_t			PositionFixIndicator;
	uint8_t			SatellitesUsed;
	float				HDOP;
	float				MSL_Altitude;
	char				MSL_Units;
	float				Geoid_Separation;
	char				Geoid_Units;
	
	uint16_t		AgeofDiffCorr;
	char				DiffRefStationID[4];
	char				CheckSum[2];	
	
}GPGGA_t;

typedef struct 
{
	uint8_t		rxBuffer[512];
	uint16_t	rxIndex;
	uint8_t		rxTmp;
	uint32_t	LastTime;	
	
	GPGGA_t		GPGGA;
	
}GPS_t;

typedef struct {
    char time[11];       // hhmmss.sss
    double latitude;     // In decimal degrees
    char lat_dir;        // 'N' or 'S'
    double longitude;    // In decimal degrees
    char lon_dir;        // 'E' or 'W'
    int fix_quality;     // GPS quality indicator
    int num_satellites;  // Number of satellites being tracked
    double hdop;         // Horizontal dilution of precision
    double altitude;     // Altitude above mean sea level
    char alt_unit;       // Altitude unit (meters)
    double geoid_height; // Height of geoid above WGS84 ellipsoid
    char geoid_unit;     // Geoid height unit (meters)
} GPGGA_Data;

typedef struct {
    char time[11];      // hhmmss.sss
    char status;        // A=active, V=void
    double latitude;    // In decimal degrees
    char lat_dir;       // 'N' or 'S'
    double longitude;   // In decimal degrees
    char lon_dir;       // 'E' or 'W'
    double speed;       // Speed over ground in knots
    double course;      // Course over ground in degrees
    char date[7];       // ddmmyy
    double magnetic_var;// Magnetic variation in degrees
    char mag_var_dir;   // Magnetic variation direction ('E' or 'W')
} GPRMC_Data;

extern GPS_t GPS;
//##################################################################################################################
int8_t	GPS_Init(void);
void	GPS_CallBack(void);
GPS_t GPS_Data_Reception(void);
void gps_print_positionning_data();
void GPS_ReadAndPrint(void);
//##################################################################################################################

#endif
