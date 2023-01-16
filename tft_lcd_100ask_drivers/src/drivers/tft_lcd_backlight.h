/**
 * @file tft_lcd_100ask_backlight.h
 */

#ifndef TFT_LCD_100ASK_BACKLIGHT_H
#define TFT_LCD_100ASK_BACKLIGHT_H

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
 * @brief Display backlight controller handle
 *
 */
typedef void * tft_lcd_100ask_backlight_h;

/**
 * @brief Configuration structure of backlight controller
 *
 * Must be passed to tft_lcd_100ask_backlight_new() for correct configuration
 */
typedef struct {
    bool pwm_control;
    bool output_invert;
    int gpio_num; // see gpio_num_t

    // Relevant only for PWM controlled backlight
    // Ignored for switch (ON/OFF) backlight control
    int timer_idx;   // ledc_timer_t
    int channel_idx; // ledc_channel_t
} tft_lcd_100ask_backlight_config_t;

/**
 * @brief Create new backlight controller
 *
 * @param[in] config Configuration structure of backlight controller
 * @return           Display backlight controller handle
 */
tft_lcd_100ask_backlight_h tft_lcd_100ask_backlight_new(const tft_lcd_100ask_backlight_config_t *config);

/**
 * @brief Set backlight
 *
 * Brightness parameter can be 0-100 for PWM controlled backlight.
 * GPIO controlled backlight (ON/OFF) is turned off witch value 0 and turned on with any positive value.
 *
 * @param bckl                   Backlight controller handle
 * @param[in] brightness_percent Brightness in [%]
 */
void tft_lcd_100ask_backlight_set(tft_lcd_100ask_backlight_h bckl, int brightness_percent);
void tft_lcd_100ask_backlight_delete(tft_lcd_100ask_backlight_h bckl);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*TFT_LCD_100ASK_BACKLIGHT_H*/
