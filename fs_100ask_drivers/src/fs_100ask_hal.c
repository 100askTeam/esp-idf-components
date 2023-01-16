/**
 * @file epd_100ask_drivers.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "fs_100ask_hal.h"
#include "drivers/internal_flash_driver.h"
#include "drivers/sd_spi_driver.h"

#ifdef CONFIG_USE_100ASK_FS_DRIVERS

/*********************
 *      DEFINES
 *********************/

#ifdef CONFIG_USE_100ASK_EXTERNAL_SDCARD_SPI
  #define SPI_SD_CARD_100ASK_PIN_MOSI           CONFIG_SPI_SD_CARD_100ASK_PIN_MOSI            /*MOSI*/
  #define SPI_SD_CARD_100ASK_PIN_MISO           CONFIG_SPI_SD_CARD_100ASK_PIN_MISO            /*MISO*/
  #define SPI_SD_CARD_100ASK_PIN_CLK            CONFIG_SPI_SD_CARD_100ASK_PIN_CLK             /*CLK*/
  #define SPI_SD_CARD_100ASK_PIN_CS             CONFIG_SPI_SD_CARD_100ASK_PIN_CS              /*CS*/

  #define SPI_SD_CARD_100ASK_MAX_TRANSFER_SIZE  CONFIG_SPI_SD_CARD_100ASK_MAX_TRANSFER_SIZE 

#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "fs_100ask_hal";

#ifdef CONFIG_USE_100ASK_EXTERNAL_SDCARD_SPI
static sdspi_100ask_driver_t storage = {
    .pin_mosi        = SPI_SD_CARD_100ASK_PIN_MOSI,
    .pin_miso        = SPI_SD_CARD_100ASK_PIN_MISO,
    .pin_sclk        = SPI_SD_CARD_100ASK_PIN_CLK,
    .pin_cs          = SPI_SD_CARD_100ASK_PIN_CS,
    .max_transfer_sz = SPI_SD_CARD_100ASK_MAX_TRANSFER_SIZE,
  };
#endif


/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void fs_100ask_hal_init(void)
{
#ifdef CONFIG_USE_100ASK_INTERNAL_FLASH
  internal_flash_100ask_init();
#elif defined(CONFIG_USE_100ASK_EXTERNAL_SDCARD_SPI)
  sdspi_100ask_init(&storage);
#else
#error "Not supported interface"
#endif
}

void fs_100ask_hal_deinit(void)
{
#ifdef CONFIG_USE_100ASK_INTERNAL_FLASH
  //internal_flash_100ask_unmount();
#elif defined(CONFIG_USE_100ASK_EXTERNAL_SDCARD_SPI)
  sdspi_100ask_unmount();
#else
#error "Not supported interface"
#endif
}

sdmmc_card_t * fs_100ask_hal_get_mount_card(void)
{
#ifdef CONFIG_USE_100ASK_INTERNAL_FLASH
  return NULL;
#elif defined(CONFIG_USE_100ASK_EXTERNAL_SDCARD_SPI)
  return sdspi_100ask_get_mount_card();
#else
#error "Not supported interface"
#endif
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
