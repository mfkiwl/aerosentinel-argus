/*
   MS5607-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
   05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
   Changelog:
     2012-05-23 - initial release.
*/
/* ============================================================================================
 MS5607-02 device SPI library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2020 João Pedro Vilas Boas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 ================================================================================================
 */

#include <DRIVERS_H/MS560702BA03/MS5607.h>
#include <stdio.h>

/* Private SPI Handler */
extern SPI_HandleTypeDef hspi4;



/* SPI Transmission Data */
static uint8_t SPITransmitData;

/* Private OSR Instantiations */
static uint8_t Pressure_OSR =  OSR_256;
static uint8_t Temperature_OSR =  OSR_256;

/* Private data holders */

/* PROM data structure */
static struct promData promData;
/* Unconpensated values structure */
static struct MS5607UncompensatedValues uncompValues;
/* Compensated values structure */
static struct MS5607Readings readings;

// Constants for the barometric formula
#define SEA_LEVEL_PRESSURE 101325.0  // Sea level standard atmospheric pressure in Pa
#define GAS_CONSTANT 8.31432         // Universal gas constant in N·m/(mol·K)
#define GRAVITY 9.80665              // Acceleration due to gravity in m/s²
#define MOLAR_MASS 0.0289644         // Molar mass of Earth's air in kg/mol
#define TEMP_LAPSE_RATE 0.0098       // Temperature lapse rate in K/m
#define STANDARD_TEMP 288.15         // Standard temperature at sea level in K
#define PASCAL_TO_HECTOPASCAL 100 	 //Divide the pressure by this number to get hPa
#define PASCAL_TO_KILOPASCAL 1000	 //Divide the pressure by this number to get kPa

void ms5607_delay_func(uint32_t period)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 64; i++)
		{
			;
		}
	}
//	HAL_Delay(period/1000);
}

/** Reset and prepare for general usage.
 * This will reset the device and perform the PROM reading to find the conversion values and if
 * the communication is working.
 */
int8_t MS5607_Init() {

  enableCSB();
  SPITransmitData = RESET_COMMAND;
  HAL_SPI_Transmit(&hspi4, &SPITransmitData, 1, 10);
  while(hspi4.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
//  HAL_Delay(3);
  ms5607_delay_func(3000);
  disableCSB();

  MS5607PromRead(&promData);

  if (promData.reserved == 0x00 || promData.reserved == 0xff) {
	 //printf("MS5607 Init fail! \n");
    return MS5607_STATE_FAILED;
  } else {
	  //printf("MS5607 Init success! \n");
    return MS5607_STATE_READY;
  }
}

/* Performs a reading on the devices PROM. */
void MS5607PromRead(struct promData *prom){
  uint8_t   address;
  uint16_t  *structPointer;

  /* As the PROM is made of 8 16bit addresses I used a pointer for acessing the data structure */
  structPointer = (uint16_t *) prom;

  for (address = 0; address < 8; address++) {
    SPITransmitData = PROM_READ(address);
    enableCSB();
    HAL_SPI_Transmit(&hspi4, &SPITransmitData, 1, 10);
    /* Receive two bytes at once and stores it directly at the structure */
    HAL_SPI_Receive(&hspi4, structPointer, 2, 10);
    disableCSB();
    structPointer++;
  }

  /* Byte swap on 16bit integers*/
  structPointer = (uint16_t *) prom;
  for (address = 0; address < 8; address++) {
    uint8_t   *toSwap = (uint8_t *) structPointer;
    uint8_t secondByte = toSwap[0];
    toSwap[0] = toSwap[1];
    toSwap[1] = secondByte;
    structPointer++;
  }
}

/* Performs a reading on the devices PROM. */
void MS5607UncompensatedRead(struct MS5607UncompensatedValues *uncompValues){

  /*Sensor reply data buffer*/
  uint8_t reply[3];

  enableCSB();
  /* Assemble the conversion command based on previously set OSR */
  SPITransmitData = CONVERT_D1_COMMAND | Pressure_OSR;
  HAL_SPI_Transmit(&hspi4, &SPITransmitData, 1, 10);
  while(hspi4.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

  if(Pressure_OSR == 0x00)
//    HAL_Delay(1);
  	ms5607_delay_func(1000);
  else if(Pressure_OSR == 0x02)
//    HAL_Delay(2);
  ms5607_delay_func(2000);
  else if(Pressure_OSR == 0x04)
//    HAL_Delay(3);
  ms5607_delay_func(3000);
  else if(Pressure_OSR == 0x06)
//    HAL_Delay(5);
  ms5607_delay_func(5000);
  else
//    HAL_Delay(10);
  ms5607_delay_func(10000);

  disableCSB();

  /* Performs the reading of the 24 bits from the ADC */

  enableCSB();

  SPITransmitData = READ_ADC_COMMAND;
  HAL_SPI_Transmit(&hspi4, &SPITransmitData, 1, 10);
  HAL_SPI_Receive(&hspi4, reply, 3, 10);

  disableCSB();

  /* Tranfer the 24bits read into a 32bit int */
  uncompValues->pressure = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];

  enableCSB();

  /* Assemble the conversion command based on previously set OSR */
  SPITransmitData = CONVERT_D2_COMMAND | Temperature_OSR;
  HAL_SPI_Transmit(&hspi4, &SPITransmitData, 1, 10);

  if(Temperature_OSR == 0x00)
//    HAL_Delay(1);
  ms5607_delay_func(1000);
  else if(Temperature_OSR == 0x02)
//    HAL_Delay(2);
  ms5607_delay_func(2000);
  else if(Temperature_OSR == 0x04)
//    HAL_Delay(3);
  ms5607_delay_func(3000);
  else if(Temperature_OSR == 0x06)
//    HAL_Delay(5);
  ms5607_delay_func(5000);
  else
//    HAL_Delay(10);
  ms5607_delay_func(10000);

  disableCSB();


  enableCSB();

  SPITransmitData = READ_ADC_COMMAND;
  HAL_SPI_Transmit(&hspi4, &SPITransmitData, 1, 10);
  HAL_SPI_Receive(&hspi4, reply, 3, 10);

  disableCSB();

  /* Assemble the conversion command based on previously set OSR */
  uncompValues->temperature = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];
}

/* Performs the data conversion according to the MS5607 datasheet */
void MS5607Convert(struct MS5607UncompensatedValues *sample, struct MS5607Readings *value){
  int32_t dT;
  int32_t TEMP;
  int64_t OFF;
  int64_t SENS;

  dT = sample->temperature - ((int32_t) (promData.tref << 8));

  TEMP = 2000 + (((int64_t) dT * promData.tempsens) >> 23);

  OFF = ((int64_t) promData.off << 17) + (((int64_t) promData.tco * dT) >> 6);
  SENS = ((int64_t) promData.sens << 16) + (((int64_t) promData.tcs * dT) >> 7);

  /**/
  if (TEMP < 2000) {
    int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
    int32_t TEMPM = TEMP - 2000;
    int64_t OFF2 = (61 * (int64_t) TEMPM * (int64_t) TEMPM) >> 4;
    int64_t SENS2 = 2 * (int64_t) TEMPM * (int64_t) TEMPM;
    if (TEMP < -1500) {
      int32_t TEMPP = TEMP + 1500;
      int32_t TEMPP2 = TEMPP * TEMPP;
      OFF2 = OFF2 + (int64_t) 15 * TEMPP2;
      SENS2 = SENS2 + (int64_t) 8 * TEMPP2;
    }
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
  }

  value->pressure = ((((int64_t) sample->pressure * SENS) >> 21) - OFF) >> 15;
  value->temperature = TEMP;
}

/* Performs the sensor reading updating the data structures */
void MS5607Update(void){
  MS5607UncompensatedRead(&uncompValues);
  MS5607Convert(&uncompValues, &readings);
}

/* Gets the temperature from the sensor reading */
double MS5607GetTemperatureC(void){
  return (double)readings.temperature/(double)100.0;
}

/* Gets the pressure from the sensor reading */
int32_t MS5607GetPressurePa(void){
  return readings.pressure;
}

/* Sets the CS pin */
void enableCSB(void){
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
}

/* Sets the CS pin */
void disableCSB(void){
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
}

/* Sets the OSR for temperature */
void MS5607SetTemperatureOSR(MS5607OSRFactors tOSR){
  Temperature_OSR = tOSR;
}

/* Sets the OSR for pressure */
void MS5607SetPressureOSR(MS5607OSRFactors pOSR){
  Pressure_OSR = pOSR;
}

// Function to calculate altitude from pressure and temperature
double calculate_altitude(double pressure, double temperature_celsius) {
	//Method 1 : Variables to tweak
    double temperature_kelvin = temperature_celsius + 273.15; // Convert temperature to Kelvin
    double altitude = (temperature_kelvin / TEMP_LAPSE_RATE) *
                      (1 - pow((pressure / SEA_LEVEL_PRESSURE), (GRAVITY * MOLAR_MASS) / (GAS_CONSTANT * TEMP_LAPSE_RATE)));

	//Method 2 : Not accurate
	//double altitude = ((((((10 * log10((pressure / 100.0) / 1013.25)) / 5.2558797) - 1) / (-6.8755856 * pow(10, -6))) / 1000) * 0.30);

	return altitude;
}


Barometer_2_Axis MS5607_ReadData(){
	Barometer_2_Axis data = {0};
	MS5607UncompensatedRead(&uncompValues);
	MS5607Convert(&uncompValues, &readings);
	data.temperature = MS5607GetTemperatureC();
	data.pressure = MS5607GetPressurePa();
	data.altitude = calculate_altitude(data.pressure, data.temperature);
	MS5607Update();
	return data;
}


void ms5607_print_barometer_data(Barometer_2_Axis *data) {
	printf("MS5607 Barometer: \n");
	printf("Pressure: %ld Pa, Temperature: %f degC, Altitude: %f meters \n", data->pressure, data->temperature, data->altitude);
    printf("----- \n");
}
