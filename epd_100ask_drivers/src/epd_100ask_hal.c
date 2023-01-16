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

#include "epd_100ask_hal.h"
#include "epd_drivers/epd_240x360_spi_driver.h"

#ifdef CONFIG_USE_100ASK_EPD

/*********************
 *      DEFINES
 *********************/
#define TAG "EPD_100ASK_DRIVERS"

#define EPD_100ASK_DISP_PIN_MOSI CONFIG_EPD_100ASK_DISP_PIN_MOSI /*SDI*/
#define EPD_100ASK_DISP_PIN_MISO CONFIG_EPD_100ASK_DISP_PIN_MISO
#define EPD_100ASK_DISP_PIN_CLK CONFIG_EPD_100ASK_DISP_PIN_CLK   /*SCLK*/
#define EPD_100ASK_DISP_PIN_CS CONFIG_EPD_100ASK_DISP_PIN_CS     /*CS*/
#define EPD_100ASK_DISP_PIN_DC CONFIG_EPD_100ASK_DISP_PIN_DC     /*D/C*/
#define EPD_100ASK_DISP_PIN_RST CONFIG_EPD_100ASK_DISP_PIN_RST   /*RESET*/
#define EPD_100ASK_DISP_PIN_BUSY CONFIG_EPD_100ASK_DISP_PIN_BUSY /*BUSY*/

#define EPD_100ASK_DISP_SPI_MODE CONFIG_EPD_100ASK_DISP_SPI_MODE
#define EPD_100ASK_DISP_SPI_FREQUENCY CONFIG_EPD_100ASK_DISP_SPI_FREQUENCY
#define EPD_100ASK_DISP_HEIGHT CONFIG_EPD_100ASK_DISP_HEIGHT
#define EPD_100ASK_DISP_WIDTH CONFIG_EPD_100ASK_DISP_WIDTH
#define EPD_100ASK_DISP_SPI_QUEUE_SIZE 7

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
#ifdef CONFIG_USE_100ASK_EPD_240X360
static epd_240x360_driver_t g_epd_t = {
    .spi_mode           = EPD_100ASK_DISP_SPI_MODE,
    .spi_clock_speed_hz = EPD_100ASK_DISP_SPI_FREQUENCY,
    .spi_host           = SPI2_HOST,
    .spi_queue_size     = EPD_100ASK_DISP_SPI_QUEUE_SIZE,
    .pin_busy           = EPD_100ASK_DISP_PIN_BUSY,
    .pin_reset          = EPD_100ASK_DISP_PIN_RST,
    .pin_dc             = EPD_100ASK_DISP_PIN_DC,
    .pin_miso           = EPD_100ASK_DISP_PIN_MOSI,
    .pin_sck            = EPD_100ASK_DISP_PIN_CLK,
    .pin_mosi           = EPD_100ASK_DISP_PIN_MOSI,
    .pin_cs             = EPD_100ASK_DISP_PIN_CS,
    .width              = EPD_100ASK_DISP_WIDTH,
    .height             = EPD_100ASK_DISP_HEIGHT,
};
#else
  #error "No device EPD defined!"
#endif

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
// Initialize the display
void epd_100ask_hal_init(void)
{
#ifdef CONFIG_USE_100ASK_EPD_240X360
  epd_240x360_init(&g_epd_t);
#endif
}

void epd_100ask_hal_deinit(void)
{
#ifdef CONFIG_USE_100ASK_EPD_240X360
  epd_240x360_deinit();
#endif
}

void epd_100ask_hal_display_clear(uint8_t color)
{
#ifdef CONFIG_USE_100ASK_EPD_240X360
  epd_240x360_display_clear(color);
#endif
}

void epd_100ask_hal_display_image(const uint8_t *data, uint16_t w, uint16_t h)
{
#ifdef CONFIG_USE_100ASK_EPD_240X360
  epd_240x360_display_image(data, w, h);
#endif
}

// partial display
void epd_100ask_hal_display_partial(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, const uint8_t *data)
{
#ifdef CONFIG_USE_100ASK_EPD_240X360
  epd_240x360_display_partial(x0, y0, w, h, data);
#endif
}

void epd_100ask_hal_refresh(EPD_LUT_TYPE lut)
{
#ifdef CONFIG_USE_100ASK_EPD_240X360
  epd_240x360_refresh(lut);
#endif
}


#endif
