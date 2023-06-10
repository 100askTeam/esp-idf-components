/**
 * @file tft_lcd_backlight.h
 */

#ifndef TFT_LCD_BACKLIGHT_H
#define TFT_LCD_BACKLIGHT_H

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#ifdef __cplusplus
extern "C" { /* extern "C" */
#endif

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/**
 * @brief Create new backlight controller
 *
 * @param[in] config Configuration structure of backlight controller
 * @return           Display backlight controller handle
 */
void tft_lcd_backlight_init(void);

/**
 * @brief Set backlight
 *
 * Brightness parameter can be 0-100 for PWM controlled backlight.
 * GPIO controlled backlight (ON/OFF) is turned off witch value 0 and turned on with any positive value.
 *
 * @param bckl                   Backlight controller handle
 * @param[in] brightness_percent Brightness in [%]
 */
void tft_lcd_backlight_set(double percent);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*TFT_LCD_BACKLIGHT_H*/
