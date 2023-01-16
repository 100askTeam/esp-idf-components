/**
 * @file epd_240x360_spi_driver.h
 *
 */

#ifndef EPD_240X360_SPI_DRIVER_H
#define EPD_240X360_SPI_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_EPD_240X360

/*********************
 *      DEFINES
 *********************/

#define EPD_100ASK_COLOR_WHITE     0xFF
#define EPD_100ASK_COLOR_BLACK     0x00


/**********************
 *      TYPEDEFS
 **********************/
typedef struct epd_240x360_driver {
    spi_device_handle_t spi;
    uint8_t spi_mode;
    int spi_host;
    int spi_queue_size;
    int spi_clock_speed_hz;
    int pin_busy;
	int pin_reset;
	int pin_dc;
	int pin_miso;
	int pin_sck;
	int pin_mosi;
	int pin_cs;
	int width;
	int height;
} epd_240x360_driver_t;

/**
 * Refresh type
**/
typedef enum {
    EPD_240x360_LUT_GC,         /* 全刷 */
    EPD_240X360_LUT_DU,         /* 局刷 */
    EPD_240X360_LUT_5S,         /* ... */
    _EPD_240X360_LUT_LAST,      /** Number of default events*/
} EPD_240X360_LUT_TYPE;


/**********************
 * GLOBAL PROTOTYPES
 **********************/

void epd_240x360_init(epd_240x360_driver_t * epd_driver);

void epd_240x360_deinit(void);

void epd_240x360_display_clear(uint8_t color);

void epd_240x360_display_image(const uint8_t * picData, uint16_t w, uint16_t h);

void epd_240x360_display_partial(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, const uint8_t * data);

void epd_240x360_refresh(EPD_240X360_LUT_TYPE lut);


/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_240X360_SPI_DRIVER_H*/