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
#include "drivers/tft_lcd_display.h"

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD

/*********************
 *      DEFINES
 *********************/
#define TAG "TFT_LCD_HAL"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
tft_lcd_display_fb_update_t update;
tft_lcd_display_fb_update_t *currentUpdate = &update;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
//Initialize the display
bool tft_lcd_100ask_hal_init(void)
{
    /*initialize screen*/
    tft_lcd_backlight_init();
    
    /*initialize screen backlight*/
    tft_lcd_display_init();
    tft_lcd_backlight_set(20.0);

    return true;

}

// LVGL library releated functions
void tft_lcd_100ask_hal_display_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    //Tell to LVGL that is ready to send another frame
    //if (pdTRUE == xSemaphoreTake(xdisplaySemaphore, portMAX_DELAY))
	{
        currentUpdate->buffer  = (tft_lcd_display_color_t *)color_map;
        currentUpdate->drv     = drv;
        currentUpdate->size    = size;
        currentUpdate->area.x1 = area->x1;
        currentUpdate->area.y1 = area->y1;
        currentUpdate->area.x2 = area->x2;
        currentUpdate->area.y2 = area->y2;

        xQueueSend(display_task_queue, &currentUpdate, portMAX_DELAY);
        //xSemaphoreGive(xdisplaySemaphore);
    }
}

void tft_lcd_100ask_hal_set_rotation(tft_lcd_100ask_rotation_t rotation)
{
    tft_lcd_display_set_rotation(rotation);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
