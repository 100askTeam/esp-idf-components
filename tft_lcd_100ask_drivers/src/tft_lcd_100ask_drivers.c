/**
 * @file tft_lcd_100ask_drivers.c
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

#include "tft_lcd_100ask_drivers.h"
#include "tft_lcd_100ask_backlight.h"
#include "st7796s.h"

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD

/*********************
 *      DEFINES
 *********************/
#define TAG "TFT_LCD_DRIVERS"


/**********************
 *      TYPEDEFS
 **********************/
#define LINE_BUFFERS (2)
#define LINE_COUNT   (20)

#define GBC_FRAME_WIDTH  160 
#define GBC_FRAME_HEIGHT 144

#define NES_FRAME_WIDTH 256
#define NES_FRAME_HEIGHT 240

#define SMS_FRAME_WIDTH 256
#define SMS_FRAME_HEIGHT 192

#define GG_FRAME_WIDTH 160
#define GG_FRAME_HEIGHT 144

#define PIXEL_MASK (0x1F) 

uint16_t *line[LINE_BUFFERS];

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
#if defined(CONFIG_USE_100ASK_SPI_TFT_LCD_ST7796S)
static ST7796S_driver_t display = {
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
#endif

// We still use menuconfig for these settings
// It will be set up during runtime in the future
#if (defined(CONFIG_SPI_TFT_LCD_100ASK_BACKLIGHT_SWITCH) || defined(CONFIG_SPI_TFT_LCD_100ASK_BACKLIGHT_PWM))
static const tft_lcd_100ask_backlight_config_t bckl_config = {
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
void * tft_lcd_100ask_drivers_init(void)
{
    ST7796S_init(&display);

    tft_lcd_100ask_backlight_h bckl_handle = tft_lcd_100ask_backlight_new(&bckl_config);
    tft_lcd_100ask_backlight_set(bckl_handle, 100);

    return bckl_handle;
}


// LVGL library releated functions
void tft_lcd_100ask_display_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map){

    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    //Set the area to print on the screen
    ST7796S_set_window(&display,area->x1,area->y1,area->x2 ,area->y2);

    //Save the buffer data and the size of the data to send
    display.current_buffer = (void *)color_map;
    display.buffer_size = size;

    //Send it
    //ST7796S_write_pixels(&display, display.current_buffer, display.buffer_size);
    ST7796S_swap_buffers(&display);

    //Tell to LVGL that is ready to send another frame
    lv_disp_flush_ready(drv);
}

void tft_lcd_100ask_clear(uint16_t color){
    ST7796S_fill_area(&display, color, 0, 0, display.display_width, display.display_height);
}

// Boot Screen Functions
uint16_t * tft_lcd_100ask_get_buffer(){
    return display.current_buffer;
}

size_t tft_lcd_100ask_get_buffer_size(){
    return display.buffer_size;
}




/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
