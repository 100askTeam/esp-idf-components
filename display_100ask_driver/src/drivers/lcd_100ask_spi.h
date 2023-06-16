/*
 * SPDX-FileCopyrightText: 2008-2023 Shenzhen Baiwenwang Technology CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LCD_100ASK_SPI_H
#define LCD_100ASK_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "driver/spi_master.h"
#include "freertos/semphr.h"

/*********************
 *      DEFINES
 *********************/

// LCD_DISPLAY Commands
#define LCD_DISPLAY_CMD_RAMCTRL           0xb0 // RAM Control
#define LCD_DISPLAY_CMD_CASET             0x2a // Column address set
#define LCD_DISPLAY_CMD_RASET             0x2b // Row address set
#define LCD_DISPLAY_CMD_RAMWR             0x2c // Memory write
#define LCD_DISPLAY_CMDLIST_END           0xff // End command (used for command list)


/**********************
 *      TYPEDEFS
 **********************/

typedef uint16_t lcd_display_color_t;

typedef struct lcd_display_transaction_data {
	struct lcd_display_driver *driver;
	bool data;
} lcd_display_transaction_data_t;

typedef struct {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
} lcd_display_area_t;

typedef struct
{
	uint32_t size;
	lv_disp_drv_t * drv;
    lcd_display_color_t *buffer;
    lcd_display_area_t   area;
} lcd_display_fb_update_t;

typedef struct lcd_display_command {
	uint8_t command;
	uint8_t wait_ms;
	uint8_t data_size;
	const uint8_t *data;
} lcd_display_command_t;

typedef enum
{
    LCD_DISPLAY_ROTATION_0 = 0,
    LCD_DISPLAY_ROTATION_90,
    LCD_DISPLAY_ROTATION_180,
    LCD_DISPLAY_ROTATION_270,
} lcd_display_rotation_t;

extern QueueHandle_t display_task_queue;
extern SemaphoreHandle_t xdisplaySemaphore;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Initialize the SPI peripheral and send the initialization sequence.
 * @param driver Screen driver structure.
 * @return True if the initialization suceed otherwise false.
 */
esp_err_t lcd_100ask_spi_init(void);

/**
 * This screen allows partial update of the screen, so we can specified which part of the windows is going to change.
 * @param driver Screen driver structure.
 * @param start_x X axis start point of the refresh zone.
 * @param start_y Y axis start point of the refresh zone. 
 * @param end_x X axis end point of the refresh zone. 
 * @param end_y Y axis end point of the refresh zone. 
 */
void lcd_100ask_spi_set_window(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);

/**
 * Set screen rotation angle
 * @param driver Screen driver structure.
 * @param rotation The rotation angles you can choose are: 90, 180, 270.
 */
void lcd_100ask_spi_set_rotation(lcd_display_rotation_t rotation);

/**********************
 *      MACROS
 **********************/

#endif /* CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* LCD_100ASK_SPI_H */
