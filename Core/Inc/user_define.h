#ifndef USER_DEFINE_H__
#define USER_DEFINE_H__

#define DEBUG_EN

#define USE_BOSCH_SENSOR_API

//#define USE_SHUTTLE20
#define USE_SHUTTLE30

//#define USE_I2C_INTERFACE
#define USE_SPI_INTERFACE

#define USE_BMI323
//#define USE_BNO055
//#define USE_BME68X

#if defined(USE_BMI323)
	//#define ACC_SELFTEST
	//#define GYRO_SELFTEST
	//#define ACC_GYRO_SELFTEST
	//#define FOC
	//#define CRT
	//#define ACC_ONLY
	//#define GYRO_ONLY
	#define ACC_GYRO
		#if defined(ACC_ONLY) || defined(ACC_GYRO)
		#define STEP_COUNTER
		#endif
	/*Only support to select one*/
	//#define ACC_RDRY_INT
	//#define GYRO_RDRY_INT

	//#define FIFO_POLL
	#define FIFO_WM_INT
	//#define SUPPORT_LOWPOWER
#elif defined(USE_BNO055)
#elif defined(USE_BME68X)
#define BME68X_SELF_TEST
//#define USE_BME68X_FORCE_MODE
//#define USE_BME68X_PARALLEL_MODE
//#define USE_BME68X_SEQUENTIAL_MODE
#endif

#endif
