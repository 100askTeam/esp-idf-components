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

#include "fs_100ask_drivers.h"
#include "sd_spi_100ask.h"

#ifdef CONFIG_USE_100ASK_FS_DRIVERS

/*********************
 *      DEFINES
 *********************/

#if defined(CONFIG_USE_100ASK_SD_SPI)
  #define SPI_SD_CARD_100ASK_PIN_MOSI           CONFIG_SPI_SD_CARD_100ASK_PIN_MOSI                     /*MOSI*/
  #define SPI_SD_CARD_100ASK_PIN_MISO           CONFIG_SPI_SD_CARD_100ASK_PIN_MISO                     /*MISO*/
  #define SPI_SD_CARD_100ASK_PIN_CLK            CONFIG_SPI_SD_CARD_100ASK_PIN_CLK                      /*CLK*/
  #define SPI_SD_CARD_100ASK_PIN_CS             CONFIG_SPI_SD_CARD_100ASK_PIN_CS                       /*CS*/

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
static const char *TAG = "fs_100ask_drivers";

#if defined(CONFIG_USE_100ASK_SD_SPI)
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
void fs_100ask_drivers_init(void)
{
    sdspi_100ask_init(&storage);
}

void fs_100ask_drivers_release(void)
{
    sdspi_100ask_unmount();
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
