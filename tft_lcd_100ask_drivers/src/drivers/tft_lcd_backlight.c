/**
 * @file tft_lcd_100ask_backlight.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "tft_lcd_backlight.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "soc/ledc_periph.h" // to invert LEDC output on IDF version < v4.3

typedef struct {
    bool pwm_control; // true: LEDC is used, false: GPIO is used
    int index;        // Either GPIO or LEDC channel
} tft_lcd_100ask_backlight_t;

//static const char *TAG = "LCD BACKLIGHT";

void tft_lcd_backlight_init(void)
{
    // Initialize backlight at 0% to avoid the lcd reset flash
    ledc_timer_config(&(ledc_timer_config_t){
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
    });
    ledc_channel_config(&(ledc_channel_config_t){
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = CONFIG_SPI_TFT_LCD_100ASK_DISP_PIN_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
    });

    ledc_fade_func_install(0);
}

void tft_lcd_backlight_set(double percent)
{
    double level = (percent / 100.0);
    level = (level > 0 ? level : 0);
    level = (level < 1.0 ? level : 1.0);

    ESP_ERROR_CHECK(ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0x1FFF * level, 50, 0));
}
