/**
 * @file tft_lcd_100ask_hal.c
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

#include "tft_lcd_100ask_hal.h"
#include "drivers/tft_lcd_backlight.h"

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_320X480
#include "drivers/tft_lcd_320x480.h"
#elif CONFIG_USE_100ASK_SPI_TFT_LCD_170X320
#include "drivers/tft_lcd_170x320.h"
#endif

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD

/*********************
 *      DEFINES
 *********************/
#define TAG "TFT_LCD_HAL"

#define LINE_BUFFERS        (2)
#define LINE_COUNT          (20)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
uint16_t *line[LINE_BUFFERS];

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_320X480
static TFT_LCD_320X480_driver_t g_display_t = {
		.pin_reset      = SPI_TFT_LCD_100ASK_DISP_PIN_RST,
		.pin_dc         = SPI_TFT_LCD_100ASK_DISP_PIN_DC,
		.pin_mosi       = SPI_TFT_LCD_100ASK_DISP_PIN_MOSI,
		.pin_sclk       = SPI_TFT_LCD_100ASK_DISP_PIN_CLK,
		.spi_host       = SPI2_HOST,
		.dma_chan       = 1,
        .display_width  = SCR_WIDTH,
		.display_height = SCR_HEIGHT,
		.buffer_size    = SCR_WIDTH * 20, // 2 buffers with 20 lines
	};
#elif CONFIG_USE_100ASK_SPI_TFT_LCD_170X320
static TFT_LCD_170X320_driver_t g_display_t = {
		.pin_reset      = SPI_TFT_LCD_100ASK_DISP_PIN_RST,
		.pin_dc         = SPI_TFT_LCD_100ASK_DISP_PIN_DC,
		.pin_mosi       = SPI_TFT_LCD_100ASK_DISP_PIN_MOSI,
		.pin_sclk       = SPI_TFT_LCD_100ASK_DISP_PIN_CLK,
		.spi_host       = SPI2_HOST,
		.dma_chan       = 1,
        .display_width  = SCR_WIDTH,
		.display_height = SCR_HEIGHT,
		.buffer_size    = SCR_WIDTH * 20, // 2 buffers with 20 lines
	};
#else
  #error "No device LCD defined!"
#endif

// We still use menuconfig for these settings
// It will be set up during runtime in the future
#if (defined(CONFIG_SPI_TFT_LCD_100ASK_BACKLIGHT_SWITCH) || defined(CONFIG_SPI_TFT_LCD_100ASK_BACKLIGHT_PWM))
static const tft_lcd_100ask_backlight_config_t backl_config = {
    .gpio_num = SPI_TFT_LCD_100ASK_DISP_PIN_BACKLIGHT,
#if defined CONFIG_SPI_TFT_LCD_100ASK_BACKLIGHT_PWM
    .pwm_control = true,
#else
    .pwm_control = false,
#endif
    .output_invert = false, // Backlight on high
    //.output_invert = true, // Backlight on low
#endif
    .timer_idx = 0,
    .channel_idx = 0 // @todo this prevents us from having two PWM controlled displays
};

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
//Initialize the display
void * tft_lcd_100ask_hal_init(void)
{
#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_320X480
    TFT_LCD_320X480_init(&g_display_t);
#elif CONFIG_USE_100ASK_SPI_TFT_LCD_170X320
    TFT_LCD_170X320_init(&g_display_t);
#endif

    tft_lcd_100ask_backlight_h backl_handle = tft_lcd_100ask_backlight_new(&backl_config);
    tft_lcd_100ask_backlight_set(backl_handle, 100);

    return backl_handle;
}


// LVGL library releated functions
void tft_lcd_100ask_hal_display_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map){

    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    //Save the buffer data and the size of the data to send
    g_display_t.current_buffer = (void *)color_map;
    g_display_t.buffer_size = size;

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_320X480
    //Set the area to print on the screen
    TFT_LCD_320X480_set_window(&g_display_t,area->x1,area->y1,area->x2 ,area->y2);

    //Send it
    //TFT_LCD_write_pixels(&g_display_t, g_display_t.current_buffer, g_display_t.buffer_size);
    TFT_LCD_320X480_swap_buffers(&g_display_t);
#elif CONFIG_USE_100ASK_SPI_TFT_LCD_170X320
    //Set the area to print on the screen
    TFT_LCD_170X320_set_window(&g_display_t,area->x1,area->y1,area->x2 ,area->y2);

    //Send it
    //TFT_LCD_write_pixels(&g_display_t, g_display_t.current_buffer, g_display_t.buffer_size);
    TFT_LCD_170X320_swap_buffers(&g_display_t);
#endif

    //Tell to LVGL that is ready to send another frame
    lv_disp_flush_ready(drv);
}

void tft_lcd_100ask_hal_clear(uint16_t color){
#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_320X480
    TFT_LCD_320X480_fill_area(&g_display_t, color, 0, 0, g_display_t.display_width, g_display_t.display_height);
#elif CONFIG_USE_100ASK_SPI_TFT_LCD_170X320
    TFT_LCD_170X320_fill_area(&g_display_t, color, 0, 0, g_display_t.display_width, g_display_t.display_height);
#endif
}

void tft_lcd_100ask_hal_set_rotation(tft_lcd_100ask_rotation_t rotation)
{
#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_320X480
    TFT_LCD_320X480_set_rotation(&g_display_t, rotation);
#elif CONFIG_USE_100ASK_SPI_TFT_LCD_170X320
    TFT_LCD_170X320_set_rotation(&g_display_t, rotation);
#endif
}

// Boot Screen Functions
uint16_t * tft_lcd_100ask_hal_get_buffer(void){
    return g_display_t.current_buffer;
}

size_t tft_lcd_100ask_hal_get_buffer_size(void){
    return g_display_t.buffer_size;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
