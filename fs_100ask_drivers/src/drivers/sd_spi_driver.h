/**
 * @file sd_spi_100ask.h
 *
 */

#ifndef SD_SPI_DRIVER_H
#define SD_SPI_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_EXTERNAL_SDCARD_SPI

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
sdmmc_card_t * sdspi_100ask_init(sdspi_100ask_driver_t *driver);

void sdspi_100ask_unmount(void);

sdmmc_card_t * sdspi_100ask_get_mount_card(void);



/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*SD_SPI_DRIVER_H*/