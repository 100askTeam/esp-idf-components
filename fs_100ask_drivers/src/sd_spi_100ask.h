/**
 * @file sd_spi_100ask.h
 *
 */

#ifndef SD_SPI_100ASK_H
#define SD_SPI_100ASK_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_SD_SPI

/*********************
 *      DEFINES
 *********************/


/**********************
 *      TYPEDEFS
 **********************/
typedef struct sdspi_100ask_driver {
	int pin_mosi;
    int pin_miso;
	int pin_sclk;
	int pin_cs;
	int max_transfer_sz;
} sdspi_100ask_driver_t;


/**********************
 * GLOBAL PROTOTYPES
 **********************/
bool sdspi_100ask_init(sdspi_100ask_driver_t *driver);

void sdspi_100ask_unmount(void);

/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*SD_SPI_100ASK_H*/