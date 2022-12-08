/**
 * @file internal_flash_100ask_drivers.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#ifdef CONFIG_USE_100ASK_INTERNAL_FLASH

#include "internal_flash_100ask_drivers.h"

/*********************
 *      DEFINES
 *********************/
#define MOUNT_POINT        CONFIG_FS_DRIVERS_MOUNT_POINT

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "INTERNAL FLASH";
// Mount path for the partition
const char *disk_path = CONFIG_FS_DRIVERS_MOUNT_POINT;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_err_t internal_flash_100ask_init(void)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    esp_err_t ret = ESP_FAIL;
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before

    // Handle of the wear levelling library instance
    wl_handle_t wl_handle_1 = WL_INVALID_HANDLE;
    ESP_LOGI(TAG, "using internal flash");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 9,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    ret = esp_vfs_fat_spiflash_mount(disk_path, "storage", &mount_config, &wl_handle_1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
#endif