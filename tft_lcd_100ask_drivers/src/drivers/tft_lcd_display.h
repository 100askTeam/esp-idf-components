/**
 * @file tft_lcd_display.h
 *
 */

#ifndef TFT_LCD_DISPLAY_H
#define TFT_LCD_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD

#include "driver/spi_master.h"
#include "freertos/semphr.h"

/*********************
 *      DEFINES
 *********************/

//TFT_LCD_DISPLAY Commands
#define TFT_LCD_DISPLAY_CMD_RAMCTRL           0xb0 // RAM Control
#define TFT_LCD_DISPLAY_CMD_CASET             0x2a // Column address set
#define TFT_LCD_DISPLAY_CMD_RASET             0x2b // Row address set
#define TFT_LCD_DISPLAY_CMD_RAMWR             0x2c // Memory write
#define TFT_LCD_DISPLAY_CMDLIST_END           0xff // End command (used for command list)


/**********************
 *      TYPEDEFS
 **********************/

typedef uint16_t tft_lcd_display_color_t;

typedef struct tft_lcd_display_transaction_data {
	struct tft_lcd_display_driver *driver;
	bool data;
} tft_lcd_display_transaction_data_t;

typedef struct {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
} tft_lcd_display_area_t;

typedef struct
{
	uint32_t size;
	lv_disp_drv_t * drv;
    tft_lcd_display_color_t *buffer;
    tft_lcd_display_area_t   area;
} tft_lcd_display_fb_update_t;

typedef struct tft_lcd_display_command {
	uint8_t command;
	uint8_t wait_ms;
	uint8_t data_size;
	const uint8_t *data;
} tft_lcd_display_command_t;

typedef enum
{
    TFT_LCD_DISPLAY_ROTATION_0 = 0,
    TFT_LCD_DISPLAY_ROTATION_90,
    TFT_LCD_DISPLAY_ROTATION_180,
    TFT_LCD_DISPLAY_ROTATION_270,
} tft_lcd_display_rotation_t;

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
bool tft_lcd_display_init(void);

/**
 * This screen allows partial update of the screen, so we can specified which part of the windows is going to change.
 * @param driver Screen driver structure.
 * @param start_x X axis start point of the refresh zone.
 * @param start_y Y axis start point of the refresh zone. 
 * @param end_x X axis end point of the refresh zone. 
 * @param end_y Y axis end point of the refresh zone. 
 */
void tft_lcd_display_set_window(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);

/**
 * Set screen rotation angle
 * @param driver Screen driver structure.
 * @param rotation The rotation angles you can choose are: 90, 180, 270.
 */
void tft_lcd_display_set_rotation(tft_lcd_display_rotation_t rotation);

/**********************
 *      MACROS
 **********************/

#endif /* CONFIG_USE_100ASK_SPI_TFT_LCD */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* EPD_100ASK_DRIVERS_H */
