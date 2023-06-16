/*
 * SPDX-FileCopyrightText: 2008-2023 Shenzhen Baiwenwang Technology CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LCD_100ASK_BACKLIGHT_H
#define LCD_100ASK_BACKLIGHT_H

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#ifdef __cplusplus
extern "C" { /* extern "C" */
#endif

#ifdef CONFIG_USE_100ASK_DISPLAY_SCREEN_BACKLIGHT

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/**
 * @brief Create new backlight controller
 *
 * @param[in] config Configuration structure of backlight controller
 * @return           Display backlight controller handle
 */
void lcd_100ask_backlight_init(void);

/**
 * @brief Set backlight
 *
 * Brightness parameter can be 0-100 for PWM controlled backlight.
 * GPIO controlled backlight (ON/OFF) is turned off witch value 0 and turned on with any positive value.
 *
 * @param bckl                   Backlight controller handle
 * @param[in] brightness_percent Brightness in [%]
 */
void lcd_100ask_backlight_set_brightness(double percent);

#endif /* CONFIG_USE_100ASK_DISPLAY_SCREEN_BACKLIGHT */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LCD_100ASK_BACKLIGHT_H */
