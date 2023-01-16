/**
 * @file sd_spi_driver.c
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

#ifdef CONFIG_USE_100ASK_EXTERNAL_SDCARD_SPI

#include "sd_spi_driver.h"

/*********************
 *      DEFINES
 *********************/
#define MOUNT_POINT        CONFIG_FS_100ASK_MOUNT_POINT

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "SD SPI";

static sdmmc_card_t *card = NULL;

// By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
// For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
// Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
static sdmmc_host_t host = SDSPI_HOST_DEFAULT();

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
sdmmc_card_t * sdspi_100ask_init(sdspi_100ask_driver_t *driver)
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    host.slot = SPI3_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = driver->pin_mosi,
        .miso_io_num = driver->pin_miso,
        .sclk_io_num = driver->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = driver->max_transfer_sz,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return false;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = driver->pin_cs;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return false;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    return card;
}

void sdspi_100ask_unmount(void)
{
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}

sdmmc_card_t * sdspi_100ask_get_mount_card(void)
{
    return card;
}
/**********************
 *   STATIC FUNCTIONS
 **********************/
#endif