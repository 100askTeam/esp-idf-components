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
#include "esp_lcd_panel_ops.h"
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
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL
    #include "drivers/lcd_100ask_esp_lcd_panel.h"
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
#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
    lcd_display_fb_update_t update;
    lcd_display_fb_update_t *currentUpdate = &update;
#endif

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
//Initialize the display
#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
esp_err_t display_100ask_hal_init(void)
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL
esp_lcd_panel_handle_t display_100ask_hal_init(void *user_data)
#endif
{    
#ifdef CONFIG_USE_100ASK_DISPLAY_SCREEN_BACKLIGHT
    /*initialize screen backlight*/
    lcd_100ask_backlight_init();
    lcd_100ask_backlight_set_brightness(20.0);
#endif

#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
    /*initialize screen*/
    esp_err_t ret = ESP_FAIL;
    ret = lcd_100ask_spi_init();

    return ret;

#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL
    esp_lcd_panel_handle_t panel_handle = NULL;
    panel_handle = lcd_100ask_esp_lcd_panel_init(user_data);
    if(panel_handle == NULL)    ESP_LOGE(TAG, "panel_handle == NULL");

    return panel_handle;
#endif

}

// LVGL library releated functions
void display_100ask_hal_lvgl_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    currentUpdate->buffer  = (lcd_display_color_t *)color_map;
    currentUpdate->drv     = drv;
    currentUpdate->size    = size;
    currentUpdate->area.x1 = area->x1;
    currentUpdate->area.y1 = area->y1;
    currentUpdate->area.x2 = area->x2;
    currentUpdate->area.y2 = area->y2;

    xQueueSend(display_task_queue, &currentUpdate, portMAX_DELAY);
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    if (panel_handle == NULL)
    {
        ESP_LOGI(TAG, "panel_handle == NULL");
        return;
    }
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
#endif
}

void display_100ask_hal_set_rotation(display_100ask_rotation_t rotation)
{
#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
    lcd_100ask_spi_set_rotation(rotation);
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL
    lcd_100ask_esp_lcd_panel_set_rotation(rotation);
#endif
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif  /* CONFIG_USE_100ASK_DISPLAY_SCREEN */
