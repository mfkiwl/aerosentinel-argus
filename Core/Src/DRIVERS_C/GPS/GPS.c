#include "DRIVERS_H/GPS/GPSConfig.h"
#include "DRIVERS_H/GPS/GPS.h"


GPS_t GPS;
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
void	GPS_CallBack(void)
{
	GPS.LastTime=HAL_GetTick();
	if(GPS.rxIndex < sizeof(GPS.rxBuffer)-2)
	{
		GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
		GPS.rxIndex++;
	}	
	HAL_UART_Receive_IT(&_GPS_UART,&GPS.rxTmp,1);
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
		str=strstr((char*)GPS.rxBuffer,"$GPGGA,");
		if(str!=NULL)
		{
			memset(&GPS.GPGGA,0,sizeof(GPS.GPGGA));
			sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS.GPGGA.UTC_Hour,&GPS.GPGGA.UTC_Min,&GPS.GPGGA.UTC_Sec,&GPS.GPGGA.UTC_MicroSec,&GPS.GPGGA.Latitude,&GPS.GPGGA.NS_Indicator,&GPS.GPGGA.Longitude,&GPS.GPGGA.EW_Indicator,&GPS.GPGGA.PositionFixIndicator,&GPS.GPGGA.SatellitesUsed,&GPS.GPGGA.HDOP,&GPS.GPGGA.MSL_Altitude,&GPS.GPGGA.MSL_Units,&GPS.GPGGA.AgeofDiffCorr,GPS.GPGGA.DiffRefStationID,GPS.GPGGA.CheckSum);
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
void gps_print_positionning_data(GPS_t *data)
{
	printf("Global Positionning System Data: \n");
    printf("UTC Time: %02d:%02d:%02d.%03d\n", data->GPGGA.UTC_Hour, data->GPGGA.UTC_Min, data->GPGGA.UTC_Sec, data->GPGGA.UTC_MicroSec);
    printf("Latitude: %.6f %c (Decimal: %.6f)\n", data->GPGGA.Latitude, data->GPGGA.NS_Indicator, data->GPGGA.LatitudeDecimal);
    printf("Longitude: %.6f %c (Decimal: %.6f)\n", data->GPGGA.Longitude, data->GPGGA.EW_Indicator, data->GPGGA.LongitudeDecimal);
    printf("Position Fix Indicator: %d\n", data->GPGGA.PositionFixIndicator);
    printf("Satellites Used: %d\n", data->GPGGA.SatellitesUsed);
    printf("HDOP: %.2f\n", data->GPGGA.HDOP);
    printf("MSL Altitude: %.2f %c\n", data->GPGGA.MSL_Altitude, data->GPGGA.MSL_Units);
    printf("Geoid Separation: %.2f %c\n", data->GPGGA.Geoid_Separation, data->GPGGA.Geoid_Units);
    printf("Age of Differential Corrections: %d\n", data->GPGGA.AgeofDiffCorr);
    printf("Differential Reference Station ID: %s\n", data->GPGGA.DiffRefStationID);
    printf("Checksum: %s\n", data->GPGGA.CheckSum);
    printf("----- \n");
}

