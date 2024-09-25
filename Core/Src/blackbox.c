/* Includes ------------------------------------------------------------------*/
#include "blackbox.h"
#include "fatfs.h"
#include "stdio.h"
#include "string.h"
#include "ff.h"


/* Private variables ---------------------------------------------------------*/
extern char SDPath[4];    // SD logical drive path
extern FATFS SDFatFS;     // File system object



/* Function implementations --------------------------------------------------*/

/**
  * @brief  Mounts the SD card file system.
  * @retval FRESULT: File function return code
  */
FRESULT SDIO_Mount_SDCard(void)
{
    FRESULT res;

    // Mount the file system
    res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK)
    {
        printf("Error mounting SD card: %d\n", res);
    }
    else
    {
        printf("SD card mounted successfully.\n");
    }

    return res;
}

/**
  * @brief  Formats the SD card with FAT32 file system.
  * @retval FRESULT: File function return code
  */
FRESULT SDIO_Format_SDCard(void)
{
    FRESULT res;
    BYTE work[512]; // Work buffer for formatting

    // Unmount the file system if mounted
    f_mount(NULL, SDPath, 0);

    // Format the card with FAT32 file system
    res = f_mkfs(SDPath, FM_ANY, 0, work, sizeof(work));
    if (res != FR_OK)
    {
        printf("Error formatting SD card: %d\n", res);
    }
    else
    {
        printf("SD card formatted successfully.\n");
    }

    // Remount the file system
    res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK)
    {
        printf("Error remounting SD card after format: %d\n", res);
    }

    return res;
}

/**
  * @brief  Gets and prints SD card information: total size and free space.
  * @retval None
  */
void SDIO_Get_SDCard_Info(void)
{
    FATFS *pfs;
    DWORD fre_clust;
    uint64_t total_sectors, free_sectors;
    uint64_t total_bytes, free_bytes;
    float total_GB, free_GB;
    FRESULT res;

    res = f_getfree(SDPath, &fre_clust, &pfs);
    if (res != FR_OK)
    {
        printf("Error getting free clusters: %d\n", res);
        return;
    }

    // Get total and free sectors
    total_sectors = (pfs->n_fatent - 2) * pfs->csize;
    free_sectors = fre_clust * pfs->csize;

    // Use fixed sector size (e.g., 512 bytes)
    uint32_t sector_size = 512;

    // Calculate total and free bytes
    total_bytes = total_sectors * sector_size;
    free_bytes = free_sectors * sector_size;

    // Convert to GB
    total_GB = (float)total_bytes / (1024.0f * 1024.0f * 1024.0f);
    free_GB = (float)free_bytes / (1024.0f * 1024.0f * 1024.0f);

    printf("SD CARD Total Size: \t%.2f GB \r\n", total_GB);
    printf("SD CARD Free Space: \t%.2f GB\r\n", free_GB);
}


/**
  * @brief  Tests file operations: create, write, read a file.
  * @retval None
  */
void SDIO_Test_FileOperations(void)
{
    FIL file;
    FRESULT res;

    // Create and open a file
    res = f_open(&file, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res == FR_OK)
    {
        // Write data to the file
        UINT bytes_written;
        const char *text = "Hello, World!";
        res = f_write(&file, text, strlen(text), &bytes_written);
        if (res == FR_OK && bytes_written == strlen(text))
        {
            printf("File written successfully.\n");
        }
        else
        {
            printf("Error writing to file: %d\n", res);
        }
        f_close(&file);
    }
    else
    {
        printf("Error opening file for writing: %d\n", res);
    }

    // Read the file back
    res = f_open(&file, "test.txt", FA_READ);
    if (res == FR_OK)
    {
        char buffer[64] = {0};
        UINT bytes_read;
        res = f_read(&file, buffer, sizeof(buffer) - 1, &bytes_read);
        if (res == FR_OK)
        {
            printf("File read successfully: %s\n", buffer);
        }
        else
        {
            printf("Error reading file: %d\n", res);
        }
        f_close(&file);
    }
    else
    {
        printf("Error opening file for reading: %d\n", res);
    }
}
