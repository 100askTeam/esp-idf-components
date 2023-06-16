/*
 * SPDX-FileCopyrightText: 2008-2023 Shenzhen Baiwenwang Technology CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifdef CONFIG_USE_100ASK_DISPLAY_SCREEN
#include "display_100ask_hal.h"

#ifdef CONFIG_USE_100ASK_DISPLAY_SCREEN_BACKLIGHT
    #include "drivers/lcd_100ask_backlight.h"
#endif

#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
    #include "drivers/lcd_100ask_spi.h"
#endif

/*********************
 *      DEFINES
 *********************/
#define TAG "LCD 100ASK HAL"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
lcd_display_fb_update_t update;
lcd_display_fb_update_t *currentUpdate = &update;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
//Initialize the display
#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
esp_err_t display_100ask_hal_init(void)
#endif
{    
#ifdef CONFIG_USE_100ASK_DISPLAY_SCREEN_BACKLIGHT
    /*initialize screen backlight*/
    lcd_100ask_backlight_init();
    lcd_100ask_backlight_set_brightness(100.0);
#endif

#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
    /*initialize screen*/
    esp_err_t ret = ESP_FAIL;
    ret = lcd_100ask_spi_init();

    return ret;

#endif

}

// LVGL library releated functions
void display_100ask_hal_lvgl_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    currentUpdate->buffer  = (lcd_display_color_t *)color_map;
    currentUpdate->drv     = drv;
    currentUpdate->size    = size;
    currentUpdate->area.x1 = area->x1;
    currentUpdate->area.y1 = area->y1;
    currentUpdate->area.x2 = area->x2;
    currentUpdate->area.y2 = area->y2;

    xQueueSend(display_task_queue, &currentUpdate, portMAX_DELAY);
}

void display_100ask_hal_set_rotation(display_100ask_rotation_t rotation)
{
    lcd_100ask_spi_set_rotation(rotation);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif  /* CONFIG_USE_100ASK_DISPLAY_SCREEN */
