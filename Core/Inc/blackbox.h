/*
 * blackbox.h
 *
 *  Created on: Sep 25, 2024
 *      Author: Utilisateur
 */

#ifndef INC_BLACKBOX_H_
#define INC_BLACKBOX_H_

#include "fatfs.h"
#include <stdio.h>
#include <string.h>


/* Function prototypes -------------------------------------------------------*/
FRESULT SDIO_Mount_SDCard(void);
FRESULT SDIO_Format_SDCard(void);
void SDIO_Get_SDCard_Info(void);
void SDIO_Test_FileOperations(void);

#endif /* INC_BLACKBOX_H_ */


