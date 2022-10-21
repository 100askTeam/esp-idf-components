/**
 * @file tft_lcd_100ask_drivers.h
 *
 */

#ifndef TFT_LCD_100ASK_DRIVERS_H
#define TFT_LCD_100ASK_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD

/* lvgl specific */
#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/
#define ST7796S_SPI_QUEUE_SIZE  2
#define HSPI_CLK_SPEED          CONFIG_SPI_TFT_LCD_100ASK_DISP_SPI_FREQUENCY

#define SCR_WIDTH               CONFIG_SPI_TFT_LCD_100ASK_DISP_WIDTH
#define SCR_HEIGHT              CONFIG_SPI_TFT_LCD_100ASK_DISP_HEIGHT

#define SPI_TFT_LCD_100ASK_DISP_PIN_MOSI               CONFIG_SPI_TFT_LCD_100ASK_DISP_PIN_MOSI
#define SPI_TFT_LCD_100ASK_DISP_PIN_CLK                CONFIG_SPI_TFT_LCD_100ASK_DISP_PIN_CLK
#define SPI_TFT_LCD_100ASK_DISP_PIN_RST                CONFIG_SPI_TFT_LCD_100ASK_DISP_PIN_RST
#define SPI_TFT_LCD_100ASK_DISP_PIN_DC                 CONFIG_SPI_TFT_LCD_100ASK_DISP_PIN_DC
#define SPI_TFT_LCD_100ASK_DISP_PIN_CS                 CONFIG_SPI_TFT_LCD_100ASK_DISP_PIN_CS
#define SPI_TFT_LCD_100ASK_DISP_SPI_MODE               CONFIG_SPI_TFT_LCD_100ASK_DISP_SPI_MODE
#define SPI_TFT_LCD_100ASK_DISP_PIN_BACKLIGHT          CONFIG_SPI_TFT_LCD_100ASK_DISP_PIN_BACKLIGHT

#define TFT_LCD_COLOR_BLACK     0x0000
#define TFT_LCD_COLOR_WHITE     0xFFFF

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 * GLOBAL PROTOTYPES
 **********************/
void *tft_lcd_100ask_drivers_init(void);

void tft_lcd_100ask_display_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);

void tft_lcd_100ask_clear(uint16_t color);

uint16_t * tft_lcd_100ask_get_buffer();

size_t tft_lcd_100ask_get_buffer_size();

/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_100ASK_DRIVERS_H*/