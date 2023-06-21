/*
 * SPDX-FileCopyrightText: 2008-2023 Shenzhen Baiwenwang Technology CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#if CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL
#include "lcd_100ask_esp_lcd_panel.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "lcd_100ask_esp_lcd_panel"

#define LCD_100ASK_SCREEN_WIDTH        CONFIG_DISPLAY_SCREEN_100ASK_WIDTH
#define LCD_100ASK_SCREEN_HEIGHT       CONFIG_DISPLAY_SCREEN_100ASK_HEIGHT

#define LCD_100ASK_SPI_FREQUENCY       CONFIG_DISPLAY_SCREEN_100ASK_SPI_FREQUENCY
#define LCD_100ASK_SPI_MODE            CONFIG_DISPLAY_SCREEN_100ASK_SPI_MODE
#define LCD_100ASK_SPI_HOST            CONFIG_DISPLAY_SCREEN_100ASK_SPI_HOST

#define LCD_100ASK_SPI_PIN_MOSI        CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_MOSI
#define LCD_100ASK_SPI_PIN_MISO        CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_MISO
#define LCD_100ASK_SPI_PIN_CLK         CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_CLK
#define LCD_100ASK_SPI_PIN_CS          CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_CS
#define LCD_100ASK_SPI_PIN_DC          CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_DC
#define LCD_100ASK_SPI_PIN_RST         CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_RST

// Bit number used to represent command and parameter
#define LCD_100ASK_CMD_BITS            (8)
#define LCD_100ASK_PARAM_BITS          (8)

/**********************
 *      TYPEDEFS
 **********************/
typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_cal; // save surrent value of LCD_CMD_COLMOD register
} lcd_100ask_panel_t;

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_bytes; // Length of data in above data array; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static esp_err_t esp_lcd_new_panel(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);
static esp_err_t panel_100ask_del(esp_lcd_panel_t *panel);
static esp_err_t panel_100ask_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_100ask_init(esp_lcd_panel_t *panel);
static esp_err_t panel_100ask_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_100ask_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_100ask_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_100ask_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_100ask_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_100ask_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);

/**********************
 *  STATIC VARIABLES
 **********************/
static uint16_t lcd_dev_xoffset = 0;
static uint16_t lcd_dev_yoffset = 0;

static esp_lcd_panel_handle_t panel_handle = NULL;

static const lcd_init_cmd_t vendor_specific_init[] = {
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320
    {0x36, {0x00}, 1},
    {0x3A, {0x05}, 1},
    {0xB2, {0x0C, 0x0C, 0x00, 0x33, 0x33}, 5},
    {0xB7, {0x35}, 1},
    {0xBB, {0x1A}, 1},
    {0xC0, {0x2C}, 1},
    {0xC2, {0x01}, 1},
    {0xC3, {0x0B}, 1},
    {0xC4, {0x20}, 1},
    {0xC6, {0x0F}, 1},
	{0xD0, {0xA4, 0xA1}, 2},
    {0xE0, {0x00, 0x03, 0x07, 0x08, 0x07, 0x15, 0x2A, 0x44, 0x42, 0x0A, 0x17, 0x18, 0x25, 0x27}, 14},
    {0XE1, {0x00, 0x03, 0x08, 0x07, 0x07, 0x23, 0x2A, 0x43, 0x42, 0x09, 0x18, 0x17, 0x25, 0x27}, 14},
    {0x21, {0}, 0},
    {0x11, {0}, 0},
    {0x29, {0}, 0},
    {0, {0}, 0xff},
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
    {0x36, {0x00}, 1},
    {0x3A, {0x05}, 1},
    {0xB2, {0x0C, 0x0C, 0x00, 0x33, 0x33}, 5},
    {0xB7, {0x35}, 1},
    {0xBB, {0x19}, 1},
    {0xC0, {0x2C}, 1},
    {0xC2, {0x01}, 1},
    {0xC3, {0x12}, 1},
    {0xC4, {0x20}, 1},
    {0xC6, {0x0F}, 1},
	{0xD0, {0xA4, 0xA1}, 2},
    {0xE0, {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23}, 14},
    {0XE1, {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}, 14},
    {0x21, {0}, 0},
    {0x11, {0}, 0},
    {0x29, {0}, 0},
    {0, {0}, 0xff},
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_320X480
    {0x11, {0x00}, 1},
    {0xF0, {0xC3}, 1},
    {0xF0, {0x96}, 1},
    {0x36, {0x48}, 1},
    {0xB4, {0x01}, 1},
    {0xB7, {0xC6}, 1},
    {0xE8, {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}, 8},
    {0xC1, {0x06}, 1},
    {0xC2, {0xA7}, 1},
    {0xC5, {0x18}, 1},
    {0xE0, {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B}, 14},
    {0XE1, {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2D, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B}, 14},
    {0xF0, {0x3C}, 1},
    {0xF0, {0x69}, 1},
    {0x3A, {0x55}, 1},
    {0x29, {0}, 0},
    {0, {0}, 0xff},
#else
	#error "Display screen init sequence is not defined for this device!"
#endif
};

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_lcd_panel_handle_t lcd_100ask_esp_lcd_panel_init(void *user_date)
{
	ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num     = LCD_100ASK_SPI_PIN_CLK,
        .mosi_io_num     = LCD_100ASK_SPI_PIN_MOSI,
        .miso_io_num     = LCD_100ASK_SPI_PIN_MISO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_100ASK_SCREEN_WIDTH * LCD_100ASK_SCREEN_HEIGHT * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_100ASK_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num         = LCD_100ASK_SPI_PIN_DC,
        .cs_gpio_num         = LCD_100ASK_SPI_PIN_CS,
        .pclk_hz             = LCD_100ASK_SPI_FREQUENCY,
        .lcd_cmd_bits        = LCD_100ASK_CMD_BITS,
        .lcd_param_bits      = LCD_100ASK_PARAM_BITS,
        .spi_mode            = LCD_100ASK_SPI_MODE,
        .trans_queue_depth   = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx            = user_date,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_100ASK_SPI_HOST, &io_config, &io_handle));

	esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_100ASK_SPI_PIN_RST,
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320 || CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
        .color_space    = ESP_LCD_COLOR_SPACE_RGB,
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_320X480
		.color_space    = ESP_LCD_COLOR_SPACE_BGR,
#else
		.color_space    = ESP_LCD_COLOR_SPACE_RGB,
#endif
        .bits_per_pixel = 16,
    };

	ESP_LOGI(TAG, "Install lcd panel driver");
	ESP_ERROR_CHECK(esp_lcd_new_panel(io_handle, &panel_config, &panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	//ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
	ESP_ERROR_CHECK(esp_lcd_panel_disp_off(panel_handle, false));

	lcd_100ask_esp_lcd_panel_set_rotation((lcd_display_rotation_t)(CONFIG_DISPLAY_SCREEN_100ASK_ROTATION / 90));

	ESP_LOGI(TAG, "Install lcd panel driver successful!");
	return panel_handle;
}

void lcd_100ask_esp_lcd_panel_set_rotation(lcd_display_rotation_t rotation)
{
	switch (rotation)
	{
	case LCD_DISPLAY_ROTATION_0:
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320
		lcd_dev_xoffset = 35;
		lcd_dev_yoffset = 0;
		esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 0;
		esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
#else
		esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
#endif
		break;
	case LCD_DISPLAY_ROTATION_90:
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 35;
		esp_lcd_panel_swap_xy(panel_handle, true);
    	esp_lcd_panel_mirror(panel_handle, true, false);
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 0;
		esp_lcd_panel_swap_xy(panel_handle, true);
    	esp_lcd_panel_mirror(panel_handle, true, false);
#else
		esp_lcd_panel_swap_xy(panel_handle, true);
    	esp_lcd_panel_mirror(panel_handle, true, true);
#endif
		break;
	case LCD_DISPLAY_ROTATION_180:
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320
		lcd_dev_xoffset = 35;
		lcd_dev_yoffset = 0;
		esp_lcd_panel_swap_xy(panel_handle, false);
    	esp_lcd_panel_mirror(panel_handle, true, true);
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 80;
		esp_lcd_panel_swap_xy(panel_handle, false);
    	esp_lcd_panel_mirror(panel_handle, true, true);
#else
		esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
#endif
		break;
	case LCD_DISPLAY_ROTATION_270:
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 35;
		esp_lcd_panel_swap_xy(panel_handle, true);
    	esp_lcd_panel_mirror(panel_handle, false, true);
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
		lcd_dev_xoffset = 80;
		lcd_dev_yoffset = 0;
		esp_lcd_panel_swap_xy(panel_handle, true);
    	esp_lcd_panel_mirror(panel_handle, false, true);
#else
		esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
#endif
		break;
	default:
		break;
	}
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static esp_err_t esp_lcd_new_panel(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    lcd_100ask_panel_t *lcd_panel = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    lcd_panel = calloc(1, sizeof(lcd_100ask_panel_t));
    ESP_GOTO_ON_FALSE(lcd_panel, ESP_ERR_NO_MEM, err, TAG, "no mem for lcd_panel panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        lcd_panel->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        lcd_panel->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        lcd_panel->colmod_cal = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        lcd_panel->colmod_cal = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    lcd_panel->io = io;
    lcd_panel->fb_bits_per_pixel = fb_bits_per_pixel;
    lcd_panel->reset_gpio_num = panel_dev_config->reset_gpio_num;
    lcd_panel->reset_level = panel_dev_config->flags.reset_active_high;
    lcd_panel->base.del = panel_100ask_del;
    lcd_panel->base.reset = panel_100ask_reset;
    lcd_panel->base.init = panel_100ask_init;
    lcd_panel->base.draw_bitmap = panel_100ask_draw_bitmap;
    lcd_panel->base.invert_color = panel_100ask_invert_color;
    lcd_panel->base.set_gap = panel_100ask_set_gap;
    lcd_panel->base.mirror = panel_100ask_mirror;
    lcd_panel->base.swap_xy = panel_100ask_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    lcd_panel->base.disp_off = panel_100ask_disp_on_off;
#else
    lcd_panel->base.disp_on_off = panel_100ask_disp_on_off;
#endif
    *ret_panel = &(lcd_panel->base);
    ESP_LOGD(TAG, "new lcd_panel panel @%p", lcd_panel);

    return ESP_OK;

err:
    if (lcd_panel) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(lcd_panel);
    }

    return ret;
}

static esp_err_t panel_100ask_del(esp_lcd_panel_t *panel)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);

    if (lcd_100ask_panel->reset_gpio_num >= 0) {
        gpio_reset_pin(lcd_100ask_panel->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del lcd panel @%p", lcd_100ask_panel);
    free(lcd_100ask_panel);
    return ESP_OK;
}

static esp_err_t panel_100ask_reset(esp_lcd_panel_t *panel)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    esp_lcd_panel_io_handle_t io = lcd_100ask_panel->io;

    // perform hardware reset
    if (lcd_100ask_panel->reset_gpio_num >= 0) {
        gpio_set_level(lcd_100ask_panel->reset_gpio_num, lcd_100ask_panel->reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(lcd_100ask_panel->reset_gpio_num, !lcd_100ask_panel->reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
    } else { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}


static esp_err_t panel_100ask_init(esp_lcd_panel_t *panel)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    esp_lcd_panel_io_handle_t io = lcd_100ask_panel->io;
#if 1
    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_MADCTL, (uint8_t[]) {
        lcd_100ask_panel->madctl_val,
    }, 1);
    esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_COLMOD, (uint8_t[]) {
        lcd_100ask_panel->colmod_cal,
    }, 1);
#endif
    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    int cmd = 0;
    while (vendor_specific_init[cmd].data_bytes != 0xff) {
        esp_lcd_panel_io_tx_param(io, vendor_specific_init[cmd].cmd, vendor_specific_init[cmd].data, vendor_specific_init[cmd].data_bytes & 0x1F);
        cmd++;
    }

    return ESP_OK;
}

static esp_err_t panel_100ask_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = lcd_100ask_panel->io;

    x_start += (lcd_100ask_panel->x_gap + lcd_dev_xoffset);
    x_end += (lcd_100ask_panel->x_gap + lcd_dev_xoffset);
    y_start += (lcd_100ask_panel->y_gap + lcd_dev_yoffset);
    y_end += (lcd_100ask_panel->y_gap + lcd_dev_yoffset);

    // define an area of frame memory where MCU can access
    esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4);
    esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4);
	
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * lcd_100ask_panel->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_DISPLAY_CMD_RAMWR, color_data, len);

    return ESP_OK;
}


static esp_err_t panel_100ask_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    esp_lcd_panel_io_handle_t io = lcd_100ask_panel->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_DISPLAY_CMD_INVON;
    } else {
        command = LCD_DISPLAY_CMD_INVOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_100ask_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    esp_lcd_panel_io_handle_t io = lcd_100ask_panel->io;
    if (mirror_x) {
        lcd_100ask_panel->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        lcd_100ask_panel->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        lcd_100ask_panel->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        lcd_100ask_panel->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_MADCTL, (uint8_t[]) {
        lcd_100ask_panel->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_100ask_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    esp_lcd_panel_io_handle_t io = lcd_100ask_panel->io;
    if (swap_axes) {
        lcd_100ask_panel->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        lcd_100ask_panel->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_DISPLAY_CMD_MADCTL, (uint8_t[]) {
        lcd_100ask_panel->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_100ask_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    lcd_100ask_panel->x_gap = x_gap;
    lcd_100ask_panel->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_100ask_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    lcd_100ask_panel_t *lcd_100ask_panel = __containerof(panel, lcd_100ask_panel_t, base);
    esp_lcd_panel_io_handle_t io = lcd_100ask_panel->io;
    int command = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        command = LCD_DISPLAY_CMD_DISPON;
    } else {
        command = LCD_DISPLAY_CMD_DISPOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
	
    return false;
}

#endif /* CONFIG_USE_100ASK_DISPLAY_SCREEN_ESP_LCD_PANEL */
