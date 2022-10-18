/**
 * @file epd_100ask_drivers.h
 *
 */

#ifndef EPD_100ASK_DRIVERS_H
#define EPD_100ASK_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

#define EPD_100ASK_COLOR_WHITE     0xFF
#define EPD_100ASK_COLOR_BLACK     0x00


/**********************
 *      TYPEDEFS
 **********************/

/**
 * Refresh type
**/
typedef enum {
    EPD_100ASK_LUT_GC,         /* 全刷 */
    EPD_100ASK_LUT_DU,         /* 局刷 */
    EPD_100ASK_LUT_5S,         /* ... */
    _EPD_100ASK_LUT_LAST,      /** Number of default events*/
} EPD_LUT_TYPE;


/**********************
 * GLOBAL PROTOTYPES
 **********************/

void epd_100ask_init(void);

void epd_100ask_display_clear(uint8_t color);

void epd_100ask_display_image(const uint8_t * picData, uint16_t w, uint16_t h);

void epd_100ask_display_partial(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, const uint8_t * data);

void epd_100ask_refresh(EPD_LUT_TYPE lut);


/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_100ASK_DRIVERS_H*/