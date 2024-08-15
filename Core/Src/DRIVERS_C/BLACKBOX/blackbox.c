/*
 * blackbox.c
 *
 *  Created on: Jul 15, 2024
 *      Author: Yassine DEHHANI
 */

#include "DRIVERS_H/BLACKBOX/blackbox.h"

static char TxBuffer[200]; // Buffer for transmission
FATFS FatFs;
FIL Fil;
FRESULT FR_Status;
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void SDIO_SDCard_Test(void) {


    do {
        // Mount the SD card
        FR_Status = SDIO_Mount_SDCard();
        if (FR_Status != FR_OK) {
        	printf("Problem mounting the SD card, check your wirings!");
        	break;
        };

        // Get and print the SD card info
        SDIO_Get_SDCard_Info();

        // Open, write to, read from, and update the file
        FR_Status = SDIO_Open_File(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
        if (FR_Status != FR_OK) break;
        SDIO_Write_File(&Fil);
        SDIO_Read_File(&Fil);
        SDIO_Update_File(&Fil);

    } while (0);

    // Unmount the SD card
    SDIO_Unmount_SDCard();
}

FRESULT SDIO_Mount_SDCard() {
    FRESULT FR_Status = f_mount(&FatFs, SDPath, 0);
    if (FR_Status != FR_OK) {
        snprintf(TxBuffer, sizeof(TxBuffer), "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
        printf(TxBuffer);
    } else {
        snprintf(TxBuffer, sizeof(TxBuffer), "SD Card Mounted Successfully! \r\n\n");
        printf(TxBuffer);
    }
    return FR_Status;
}

void SDIO_Get_SDCard_Info() {

	//FRESULT INFO_Status;

	f_getfree("", &fre_clust, &pfs);
//    if (INFO_Status != FR_OK) {
//    	printf("Problem checking disk size.");
//    	return;
//    };


    total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    float total_GB = (float)total / (1024.0 * 1024.0);
    printf( "SD CARD Total Size: \t%.2f GB \r\n", total_GB);



    free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
    float free_space_GB = (float)free_space / (1024.0 * 1024.0);
    printf("SD CARD Free Space: \t%.2f GB\r\n", free_space_GB);
}

FRESULT SDIO_Open_File(FIL *Fil, const char *filename, BYTE mode) {
    FRESULT FR_Status = f_open(Fil, filename, mode);
    if (FR_Status != FR_OK) {
        snprintf(TxBuffer, sizeof(TxBuffer), "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
        printf(TxBuffer);
    } else {
        snprintf(TxBuffer, sizeof(TxBuffer), "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
        printf(TxBuffer);
    }
    return FR_Status;
}

void SDIO_Write_File(FIL *Fil) {
    char RW_Buffer[200];
    // Write data to the text file using f_puts() and f_write()
    f_puts("Hello! From STM32 To SD Card Over SDMMC, Using f_puts()\n", Fil);
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDMMC, Using f_write()\r\n");
    UINT WWC;
    f_write(Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    f_close(Fil);
}

void SDIO_Read_File(FIL *Fil) {
    char RW_Buffer[200];
    UINT RWC;

    // Open the file for reading
    FRESULT FR_Status = f_open(Fil, "MyTextFile.txt", FA_READ);
    if (FR_Status != FR_OK) {
        snprintf(TxBuffer, sizeof(TxBuffer), "Error! While Opening (MyTextFile.txt) File For Read.. \r\n");
        printf(TxBuffer);
        return;
    }

    // Read data from the text file using f_gets() and f_read()
    f_gets(RW_Buffer, sizeof(RW_Buffer), Fil);
    snprintf(TxBuffer, sizeof(TxBuffer), "Data Read From (MyTextFile.txt) Using f_gets():%.150s", RW_Buffer);
    printf(TxBuffer);

    f_read(Fil, RW_Buffer, f_size(Fil), &RWC);
    snprintf(TxBuffer, sizeof(TxBuffer), "Data Read From (MyTextFile.txt) Using f_read():%.150s", RW_Buffer);
    printf(TxBuffer);
    f_close(Fil);

    snprintf(TxBuffer, sizeof(TxBuffer), "File Closed! \r\n\n");
    printf(TxBuffer);
}

void SDIO_Update_File(FIL *Fil) {
    char RW_Buffer[200];
    UINT RWC;

    // Open the existing file for update
    FRESULT FR_Status = f_open(Fil, "MyTextFile.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(Fil, f_size(Fil)); // Move the file pointer to the end
    if (FR_Status != FR_OK) {
        snprintf(TxBuffer, sizeof(TxBuffer), "Error! While Opening (MyTextFile.txt) File For Update.. \r\n");
        printf(TxBuffer);
        return;
    }

    // Write new line of text data to the file
    f_puts("This New Line Was Added During File Update!\r\n", Fil);
    f_close(Fil);
    memset(RW_Buffer, '\0', sizeof(RW_Buffer)); // Clear the buffer

    // Read the contents of the text file after the update
    FR_Status = f_open(Fil, "MyTextFile.txt", FA_READ);
    f_read(Fil, RW_Buffer, f_size(Fil), &RWC);
    snprintf(TxBuffer, sizeof(TxBuffer), "Data Read From (MyTextFile.txt) After Update:\r\n%.150s", RW_Buffer);
    printf(TxBuffer);
    f_close(Fil);
}

void SDIO_Unmount_SDCard(void) {
    FRESULT FR_Status = f_mount(NULL, "", 0);
    if (FR_Status != FR_OK) {
        snprintf(TxBuffer, sizeof(TxBuffer), "\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
        printf(TxBuffer);
    } else {
        snprintf(TxBuffer, sizeof(TxBuffer), "\r\nSD Card Un-mounted Successfully! \r\n");
        printf(TxBuffer);
    }
}

