/*
 * blackbox.h
 *
 *  Created on: Jul 15, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_BLACKBOX_BLACKBOX_H_
#define INC_DRIVERS_H_BLACKBOX_BLACKBOX_H_

#include "fatfs.h"
#include <stdio.h>
#include <string.h>


void SDIO_SDCard_Test(void);
FRESULT SDIO_Mount_SDCard(FATFS *FatFs, const char *SDPath);
void SDIO_Get_SDCard_Info(FATFS *FatFs, FATFS **FS_Ptr);
FRESULT SDIO_Open_File(FIL *Fil, const char *filename, BYTE mode);
void SDIO_Write_File(FIL *Fil);
void SDIO_Read_File(FIL *Fil);
void SDIO_Update_File(FIL *Fil);
void SDIO_Unmount_SDCard(void);

#endif /* INC_DRIVERS_H_BLACKBOX_BLACKBOX_H_ */
