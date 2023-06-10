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

#define SPI_BUFFER_SIZE       (20 * 320)

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
    sdmmc_host_t host_config = SDSPI_HOST_DEFAULT();
    host_config.slot = SPI2_HOST;
    host_config.max_freq_khz = SDMMC_FREQ_DEFAULT;
    host_config.do_transaction = &sdspi_host_do_transaction;
    esp_err_t err;

    // Starting with 4.2.0 we have to initialize the SPI bus ourselves
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = SPI2_HOST;
    slot_config.gpio_cs = driver->pin_cs;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = driver->pin_mosi,
        .miso_io_num = driver->pin_miso,
        .sclk_io_num = driver->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz= SPI_BUFFER_SIZE * 2 * sizeof(uint16_t),
    };
    err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) // check but do not abort, let esp_vfs_fat_sdspi_mount decide
        ESP_LOGE(TAG, "SPI bus init failed (0x%x)\n", err);
#else
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = driver->pin_miso;
    slot_config.gpio_mosi = driver->pin_mosi;
    slot_config.gpio_sck = driver->pin_sclk;
    slot_config.gpio_cs = driver->pin_cs;
    slot_config.dma_channel = 1;
    #define esp_vfs_fat_sdspi_mount esp_vfs_fat_sdmmc_mount
#endif

    esp_vfs_fat_mount_config_t mount_config = {.max_files = 5};

    err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host_config, &slot_config, &mount_config, &card);
    if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_RESPONSE || err == ESP_ERR_INVALID_CRC)
    {
        ESP_LOGW(TAG, "SD Card mounting failed (0x%x), retrying at lower speed...\n", err);
        host_config.max_freq_khz = SDMMC_FREQ_PROBING;
        err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host_config, &slot_config, &mount_config, &card);
    }

     ESP_LOGE(TAG, "SPI bus init OK");

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
