/*
 * SPDX-FileCopyrightText: 2008-2023 Shenzhen Baiwenwang Technology CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LCD_100ASK_ESP_LCD_PANEL_H
#define LCD_100ASK_ESP_LCD_PANEL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/
#define LCD_DISPLAY_CMD_RAMCTRL           0xb0 // RAM Control
#define LCD_DISPLAY_CMD_CASET             0x2a // Column address set
#define LCD_DISPLAY_CMD_RASET             0x2b // Row address set
#define LCD_DISPLAY_CMD_RAMWR             0x2c // Memory write
#define LCD_DISPLAY_CMDLIST_END           0xff // End command (used for command list)
#define LCD_DISPLAY_CMD_MADCTL            0x36 // Memory data access control
#define LCD_DISPLAY_CMD_SLPOUT            0x11 // Exit sleep mode
#define LCD_DISPLAY_CMD_COLMOD            0x3A // Defines the format of RGB picture data
#define LCD_DISPLAY_CMD_SWRESET           0x01 // Software reset registers (the built-in frame buffer is not affected)
#define LCD_DISPLAY_CMD_INVOFF            0x20 // Recover from display inversion mode
#define LCD_DISPLAY_CMD_INVON             0x21 // Go into display inversion mode
#define LCD_DISPLAY_CMD_DISPOFF           0x28 // Display off (disable frame buffer output)
#define LCD_DISPLAY_CMD_DISPON            0x29 // Display on (enable frame buffer output)


/**********************
 *      TYPEDEFS
 **********************/
typedef enum
{
    LCD_DISPLAY_ROTATION_0 = 0,
    LCD_DISPLAY_ROTATION_90,
    LCD_DISPLAY_ROTATION_180,
    LCD_DISPLAY_ROTATION_270,
} lcd_display_rotation_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

esp_lcd_panel_handle_t lcd_100ask_esp_lcd_panel_init(void *user_date);

void lcd_100ask_esp_lcd_panel_set_rotation(lcd_display_rotation_t rotation);

/**********************
 *      MACROS
 **********************/

#endif /* CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* LCD_100ASK_ESP_LCD_PANEL_H */
