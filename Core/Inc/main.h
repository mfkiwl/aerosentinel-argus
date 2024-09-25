/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_CS_Pin GPIO_PIN_3
#define SPI4_CS_GPIO_Port GPIOE
#define MS5607_ON_Pin GPIO_PIN_13
#define MS5607_ON_GPIO_Port GPIOC
#define SD_Detect_Pin GPIO_PIN_1
#define SD_Detect_GPIO_Port GPIOA
#define ADXL375_ON_Pin GPIO_PIN_3
#define ADXL375_ON_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define MPL311_ON_Pin GPIO_PIN_14
#define MPL311_ON_GPIO_Port GPIOE
#define HEARTBEAT_Pin GPIO_PIN_15
#define HEARTBEAT_GPIO_Port GPIOB
#define BNO055_ON_Pin GPIO_PIN_15
#define BNO055_ON_GPIO_Port GPIOA
#define BNO055_RST_Pin GPIO_PIN_3
#define BNO055_RST_GPIO_Port GPIOD
#define GPS_RST_Pin GPIO_PIN_4
#define GPS_RST_GPIO_Port GPIOD
#define BNO086_RST_Pin GPIO_PIN_5
#define BNO086_RST_GPIO_Port GPIOB
#define BNO086_ON_Pin GPIO_PIN_1
#define BNO086_ON_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
