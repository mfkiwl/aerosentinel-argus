#include "DRIVERS_H/GPS/GPSConfig.h"
#include "DRIVERS_H/GPS/GPS.h"



GPS_t GPS;
GPGGA_Data gpgga_data;
GPRMC_Data gprmc_data;
//##################################################################################################################
double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}
//##################################################################################################################
int8_t GPS_Init(void)
{
	GPS.rxIndex=0;
	HAL_StatusTypeDef status = HAL_UART_Receive_IT(&_GPS_UART,&GPS.rxTmp,1);

	return (status == HAL_OK) ? 0 : 1;
}
//##################################################################################################################

int parse_gpgga(const char* nmea, GPGGA_Data* data) {
    if (strncmp(nmea, "$GNGGA", 6) != 0) {
        return 0; // Not a GPGGA sentence
    }

    // Parse the NMEA sentence
    sscanf(nmea,
           "$GNGGA,%10[^,],%lf,%c,%lf,%c,%d,%d,%lf,%lf,%c,%lf,%c",
           data->time,
           &data->latitude,
           &data->lat_dir,
           &data->longitude,
           &data->lon_dir,
           &data->fix_quality,
           &data->num_satellites,
           &data->hdop,
           &data->altitude,
           &data->alt_unit,
           &data->geoid_height,
           &data->geoid_unit);

    // Convert latitude and longitude from degrees, minutes to decimal degrees
    data->latitude = (int)(data->latitude / 100) + fmod(data->latitude, 100) / 60.0;
    if (data->lat_dir == 'S') {
        data->latitude = -data->latitude;
    }

    data->longitude = (int)(data->longitude / 100) + fmod(data->longitude, 100) / 60.0;
    if (data->lon_dir == 'W') {
        data->longitude = -data->longitude;
    }

    return 1;
}

int parse_gprmc(const char* nmea, GPRMC_Data* data) {
    if (strncmp(nmea, "$GNRMC", 6) != 0) {
        return 0; // Not a GPRMC sentence
    }

    // Parse the NMEA sentence
    sscanf(nmea,
           "$GNRMC,%10[^,],%c,%lf,%c,%lf,%c,%lf,%lf,%6[^,],%lf,%c",
           data->time,
           &data->status,
           &data->latitude,
           &data->lat_dir,
           &data->longitude,
           &data->lon_dir,
           &data->speed,
           &data->course,
           data->date,
           &data->magnetic_var,
           &data->mag_var_dir);

    // Convert latitude and longitude from degrees, minutes to decimal degrees
    data->latitude = (int)(data->latitude / 100) + fmod(data->latitude, 100) / 60.0;
    if (data->lat_dir == 'S') {
        data->latitude = -data->latitude;
    }

    data->longitude = (int)(data->longitude / 100) + fmod(data->longitude, 100) / 60.0;
    if (data->lon_dir == 'W') {
        data->longitude = -data->longitude;
    }

    return 1;
}

void print_gpgga(const GPGGA_Data* data) {
    printf("GNGGA Data:\n");
    printf("  Time: %s\n", data->time);
    printf("  Latitude: %.6f %c\n", data->latitude, data->lat_dir);
    printf("  Longitude: %.6f %c\n", data->longitude, data->lon_dir);
    printf("  Fix Quality: %d\n", data->fix_quality);
    printf("  Number of Satellites: %d\n", data->num_satellites);
    printf("  HDOP: %.2f\n", data->hdop);
    printf("  Altitude: %.2f %c\n", data->altitude, data->alt_unit);
    printf("  Geoid Height: %.2f %c\n", data->geoid_height, data->geoid_unit);
}


void print_gprmc(const GPRMC_Data* data) {
    printf("GNRMC Data:\n");
    printf("  Time: %s\n", data->time);
    printf("  Status: %c\n", data->status);
    printf("  Latitude: %.6f %c\n", data->latitude, data->lat_dir);
    printf("  Longitude: %.6f %c\n", data->longitude, data->lon_dir);
    printf("  Speed (knots): %.2f\n", data->speed);
    printf("  Course (degrees): %.2f\n", data->course);
    printf("  Date: %s\n", data->date);
    printf("  Magnetic Variation: %.2f %c\n", data->magnetic_var, data->mag_var_dir);
}

void GPS_CallBack(void)
{
    GPS.LastTime = HAL_GetTick();

    // Store the received byte into the buffer if there's space
    if(GPS.rxIndex < sizeof(GPS.rxBuffer) - 2)
    {
        GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
        GPS.rxIndex++;

        // Check if the received byte is a newline character
        if (GPS.rxTmp == '\n')
        {
            // Null-terminate the received string
            GPS.rxBuffer[GPS.rxIndex] = '\0';

            // Print the received NMEA sentence
            //printf("Received GPS Data: %s\n", GPS.rxBuffer);

            // Determine the type of NMEA sentence and parse accordingly
            if (strncmp((const char*)GPS.rxBuffer, "$GNGGA", 6) == 0) {
                GPGGA_Data gpgga_data;
                if (parse_gpgga((const char*)GPS.rxBuffer, &gpgga_data)) {
//                	printf("GPGGA parsing success: \n");
                    print_gpgga(&gpgga_data);
                    printf("// -------------------- // \n");
                }
            } else if (strncmp((const char*)GPS.rxBuffer, "$GNRMC", 6) == 0) {
                GPRMC_Data gprmc_data;
                if (parse_gprmc((const char*)GPS.rxBuffer, &gprmc_data)) {
//                	printf("GPRMC parsing success: \n");
                    print_gprmc(&gprmc_data);
                    printf("// -------------------- // \n");
                }
            }

            // Reset the index for the next message
            GPS.rxIndex = 0;
        }
    }

    // Restart UART reception in interrupt mode
    HAL_UART_Receive_IT(&_GPS_UART, &GPS.rxTmp, 1);
}


// Override the weak implementation of the callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Check if the callback is for UART8
    if (huart->Instance == UART8)
    {
        // Call the GPS callback function
        GPS_CallBack();
    }
}

//##################################################################################################################
GPS_t GPS_Data_Reception(void)
{
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{
		char	*str;
		#if (_GPS_DEBUG==1)
		printf("%s",GPS.rxBuffer);
		#endif
		str=strstr((char*)GPS.rxBuffer,"$GNGGA,");
		if(str!=NULL)
		{
			memset(&GPS.GPGGA,0,sizeof(GPS.GPGGA));
			sscanf(str,"$GNGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS.GPGGA.UTC_Hour,&GPS.GPGGA.UTC_Min,&GPS.GPGGA.UTC_Sec,&GPS.GPGGA.UTC_MicroSec,&GPS.GPGGA.Latitude,&GPS.GPGGA.NS_Indicator,&GPS.GPGGA.Longitude,&GPS.GPGGA.EW_Indicator,&GPS.GPGGA.PositionFixIndicator,&GPS.GPGGA.SatellitesUsed,&GPS.GPGGA.HDOP,&GPS.GPGGA.MSL_Altitude,&GPS.GPGGA.MSL_Units,&GPS.GPGGA.AgeofDiffCorr,GPS.GPGGA.DiffRefStationID,GPS.GPGGA.CheckSum);
			if(GPS.GPGGA.NS_Indicator==0)
				GPS.GPGGA.NS_Indicator='-';
			if(GPS.GPGGA.EW_Indicator==0)
				GPS.GPGGA.EW_Indicator='-';
			if(GPS.GPGGA.Geoid_Units==0)
				GPS.GPGGA.Geoid_Units='-';
			if(GPS.GPGGA.MSL_Units==0)
				GPS.GPGGA.MSL_Units='-';
			GPS.GPGGA.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Latitude);
			GPS.GPGGA.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Longitude);
		}
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&_GPS_UART,&GPS.rxTmp,1);

	return GPS;
}




//##################################################################################################################
// Function to print the GPS position data
void gps_print_positionning_data()
{
	printf("Global Positionning System Data: \n");
    printf("UTC Time: %02d:%02d:%02d.%03d\n", GPS.GPGGA.UTC_Hour, GPS.GPGGA.UTC_Min, GPS.GPGGA.UTC_Sec, GPS.GPGGA.UTC_MicroSec);
    printf("Latitude: %.6f %c (Decimal: %.6f)\n", GPS.GPGGA.Latitude, GPS.GPGGA.NS_Indicator, GPS.GPGGA.LatitudeDecimal);
    printf("Longitude: %.6f %c (Decimal: %.6f)\n", GPS.GPGGA.Longitude, GPS.GPGGA.EW_Indicator, GPS.GPGGA.LongitudeDecimal);
    printf("Position Fix Indicator: %d\n", GPS.GPGGA.PositionFixIndicator);
    printf("Satellites Used: %d\n", GPS.GPGGA.SatellitesUsed);
    printf("HDOP: %.2f\n", GPS.GPGGA.HDOP);
    printf("MSL Altitude: %.2f %c\n", GPS.GPGGA.MSL_Altitude, GPS.GPGGA.MSL_Units);
    printf("Geoid Separation: %.2f %c\n", GPS.GPGGA.Geoid_Separation, GPS.GPGGA.Geoid_Units);
    printf("Age of Differential Corrections: %d\n", GPS.GPGGA.AgeofDiffCorr);
    printf("Differential Reference Station ID: %s\n", GPS.GPGGA.DiffRefStationID);
    printf("Checksum: %s\n", GPS.GPGGA.CheckSum);
    printf("----- \n");
}


