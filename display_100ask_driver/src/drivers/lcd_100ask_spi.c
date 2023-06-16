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
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_log.h"

#if CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE
#include "lcd_100ask_spi.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "display_driver"

#define LCD_100ASK_SPI_FREQUENCY   CONFIG_DISPLAY_SCREEN_100ASK_SPI_FREQUENCY
#define LCD_100ASK_SPI_MODE        CONFIG_DISPLAY_SCREEN_100ASK_SPI_MODE
#define LCD_100ASK_SPI_HOST        CONFIG_DISPLAY_SCREEN_100ASK_SPI_HOST

#define LCD_100ASK_PIN_MOSI        CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_MOSI
#define LCD_100ASK_PIN_MISO        CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_MISO
#define LCD_100ASK_PIN_CLK         CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_CLK
#define LCD_100ASK_PIN_CS          CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_CS
#define LCD_100ASK_PIN_DC          CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_DC
#define LCD_100ASK_PIN_RST         CONFIG_DISPLAY_SCREEN_100ASK_SPI_PIN_RST

#define SPI_TRANSACTION_COUNT (8)
#define SPI_BUFFER_COUNT      (5)
#define SPI_TASK_PRIORITY 	  (10)

#ifdef CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320
	#define SPI_BUFFER_SIZE       (20 * 320)
#elif  CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
	#define SPI_BUFFER_SIZE       (20 * 240)
#elif  CONFIG_USE_100ASK_DISPLAY_SCREEN_320X480
	#define SPI_BUFFER_SIZE       (10 * 480)
#endif

#define lcd_send_data(buffer, length) spi_queue_transaction(buffer, length, 3)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lcd_100ask_reset(void);
static void lcd_100ask_config(lcd_display_rotation_t rotation);
static void lcd_100ask_pre_cb(spi_transaction_t *transaction);
static void lcd_100ask_display_task(void *arg);

static void lcd_100ask_send_cmd(const lcd_display_command_t *command);
static void lcd_100ask_send_multi( const lcd_display_command_t *sequence);

static inline uint16_t *spi_get_buffer(void);
static inline void spi_queue_transaction(const void *data, size_t length, uint32_t type);
IRAM_ATTR static void spi_task(void *arg);


/**********************
 *  STATIC VARIABLES
 **********************/
static uint16_t lcd_dev_xoffset = 0;
static uint16_t lcd_dev_yoffset = 0;

static spi_device_handle_t spi_dev;
static QueueHandle_t spi_transactions;
static QueueHandle_t spi_buffers;

QueueHandle_t display_task_queue;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_err_t lcd_100ask_spi_init(void){
	esp_err_t ret;

	spi_transactions = xQueueCreate(SPI_TRANSACTION_COUNT, sizeof(spi_transaction_t *));
    spi_buffers = xQueueCreate(SPI_BUFFER_COUNT, sizeof(uint16_t *));

	while (uxQueueSpacesAvailable(spi_transactions))
    {
        void *trans = malloc(sizeof(spi_transaction_t));
        xQueueSend(spi_transactions, &trans, portMAX_DELAY);
    }

    while (uxQueueSpacesAvailable(spi_buffers))
    {
        void *buffer = heap_caps_calloc(1, (SPI_BUFFER_SIZE * 2), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        xQueueSend(spi_buffers, &buffer, portMAX_DELAY);
    }

    /* Set-Up SPI BUS */
    spi_bus_config_t buscfg = {
		.mosi_io_num    = LCD_100ASK_PIN_MOSI,
		.miso_io_num    = -1,
		.sclk_io_num    = LCD_100ASK_PIN_CLK,
		.quadwp_io_num  = -1,
		.quadhd_io_num  = -1,
		.max_transfer_sz= SPI_BUFFER_SIZE * 2 * sizeof(lcd_display_color_t), 
	};

    /* Configure SPI BUS */
    spi_device_interface_config_t devcfg = {
		.clock_speed_hz = LCD_100ASK_SPI_FREQUENCY,
		.mode           = LCD_100ASK_SPI_MODE,
		.spics_io_num   = LCD_100ASK_PIN_CS,
		.queue_size     = SPI_TRANSACTION_COUNT,
		.pre_cb         = lcd_100ask_pre_cb,
		.flags 			= SPI_DEVICE_NO_DUMMY,
	};

	ESP_LOGI(TAG, "Initializing SPI bus...");
	ret = spi_bus_initialize(SPI2_HOST, &buscfg, (spi_dma_chan_t)SPI_DMA_CH_AUTO);
    if(ret != ESP_OK || ret != ESP_ERR_INVALID_STATE){
        ESP_LOGE(TAG,"SPI Bus initialization failed.");
    }

	ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev);
	if(ret != ESP_OK){
        ESP_LOGE(TAG,"SPI Bus add device failed.");
    }

	if (xTaskCreatePinnedToCore(&spi_task, "lcd_spi", (1.5 * 1024), NULL, (SPI_TASK_PRIORITY - 1), NULL, 1) != pdPASS)
    {
		ESP_LOGE(TAG, "Task[%s] creation failed!", "lcd_spi");
        return ESP_FAIL;
	}

    ESP_LOGI(TAG,"SPI Bus configured correctly.");

    /* Set the RESET and DC PIN */ 
    gpio_pad_select_gpio(LCD_100ASK_PIN_RST);
    gpio_pad_select_gpio(LCD_100ASK_PIN_DC);
    gpio_set_direction(LCD_100ASK_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_100ASK_PIN_DC, GPIO_MODE_OUTPUT);

    /* Set the screen configuration */
    lcd_100ask_reset();

	/*initialize screen*/
	lcd_100ask_config((lcd_display_rotation_t)(CONFIG_DISPLAY_SCREEN_100ASK_ROTATION / 90));
	
	if (xTaskCreatePinnedToCore(&lcd_100ask_display_task, "lcd_display", (3 * 1024), NULL, 5, NULL, 1) != pdPASS)
    {
		ESP_LOGE(TAG, "Task[%s] creation failed!", "lcd_display");
        return ESP_FAIL;
	}

    ESP_LOGI(TAG, "Display configured and ready to work.");

    return ESP_OK;
}

void lcd_100ask_spi_set_window(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y){
	uint8_t caset[4];
	uint8_t raset[4];

	caset[0] = (uint8_t)((start_x + lcd_dev_xoffset) >> 8) & 0xFF;
	caset[1] = (uint8_t)((start_x + lcd_dev_xoffset) & 0xff);
	caset[2] = (uint8_t)((end_x + lcd_dev_xoffset) >> 8) & 0xFF;
	caset[3] = (uint8_t)((end_x + lcd_dev_xoffset) & 0xff) ;
	
	raset[0] = (uint8_t)((start_y + lcd_dev_yoffset) >> 8) & 0xFF;
	raset[1] = (uint8_t)((start_y + lcd_dev_yoffset) & 0xff);
	raset[2] = (uint8_t)((end_y + lcd_dev_yoffset) >> 8) & 0xFF;
	raset[3] = (uint8_t)((end_y + lcd_dev_yoffset) & 0xff);

	lcd_display_command_t sequence[] = {
		{LCD_DISPLAY_CMD_CASET, 0, 4, caset},
		{LCD_DISPLAY_CMD_RASET, 0, 4, raset},
		{LCD_DISPLAY_CMD_RAMWR, 0, 0, NULL},
		{LCD_DISPLAY_CMDLIST_END, 0, 0, NULL},
	};

	lcd_100ask_send_multi(sequence);
}

void lcd_100ask_spi_set_rotation(lcd_display_rotation_t rotation)
{
	lcd_100ask_reset();
    lcd_100ask_config(rotation);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
IRAM_ATTR
static void spi_task(void *arg)
{
	spi_transaction_t *t;

    while (spi_device_get_trans_result(spi_dev, &t, portMAX_DELAY) == ESP_OK)
    {
        if ((int)t->user & 2)
            xQueueSend(spi_buffers, &t->tx_buffer, 0);
        xQueueSend(spi_transactions, &t, 0);
    }

    // This should never happen
    vTaskDelete(NULL);
}

static void lcd_100ask_display_task(void *arg)
{
    display_task_queue = xQueueCreate(1, sizeof(lcd_display_fb_update_t *));
	if(!display_task_queue)	ESP_LOGE(TAG, "display_task_queue == NULL");

    while (1)
    {
		lcd_display_fb_update_t *update;

		xQueuePeek(display_task_queue, &update, portMAX_DELAY);

		lcd_display_area_t * area = &update->area;

		//Set the area to print on the screen
		lcd_100ask_spi_set_window(area->x1, area->y1, area->x2, area->y2);

		if(update->size > SPI_BUFFER_SIZE)
		{
			uint16_t mod  = update->size % SPI_BUFFER_SIZE;
			uint16_t count = update->size / SPI_BUFFER_SIZE;

			uint16_t i;
			for(i = 0; i < count; i++)
			{
				lcd_send_data((update->buffer + (i * SPI_BUFFER_SIZE)), SPI_BUFFER_SIZE * sizeof(lcd_display_color_t));
			}
			if(mod > 0)	lcd_send_data((update->buffer + (i * SPI_BUFFER_SIZE)), mod * sizeof(lcd_display_color_t));
		}
		else	lcd_send_data(update->buffer, update->size * sizeof(lcd_display_color_t));

		lv_disp_flush_ready(update->drv);
		//ESP_LOGI(TAG,"area->x1: %d, area->y1: %d, area->x2: %d, area->y2: %d, size: %d", area->x1, area->y1, area->x2, area->y2, update->size);

		xQueueReceive(display_task_queue, &update, portMAX_DELAY);
	}
}

static void lcd_100ask_reset(void) {
	gpio_set_level(LCD_100ASK_PIN_RST, 0);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	gpio_set_level(LCD_100ASK_PIN_RST, 1);
	vTaskDelay(20 / portTICK_PERIOD_MS);
}

static inline uint16_t *spi_get_buffer(void)
{
    uint16_t *buffer;
    if (xQueueReceive(spi_buffers, &buffer, pdMS_TO_TICKS(2500)) != pdTRUE)
        ESP_LOGE(TAG, "xQueueReceive error!");
    return buffer;
}

static inline void spi_queue_transaction(const void *data, size_t length, uint32_t type)
{
    spi_transaction_t *t;

    if (!data || length < 1)
        return;

    xQueueReceive(spi_transactions, &t, portMAX_DELAY);

    *t = (spi_transaction_t){
        .tx_buffer = NULL,
        .length = length * 8, // In bits
        .user = (void *)type,
        .flags = 0,
    };

    if (type & 2)
    {
        t->tx_buffer = data;
    }
    else if (length < 5)
    {
        memcpy(t->tx_data, data, length);
        t->flags = SPI_TRANS_USE_TXDATA;
    }
    else
    {
        t->tx_buffer = memcpy(spi_get_buffer(), data, length);
		t->user = (void *)(type | 2);
    }

    if (spi_device_queue_trans(spi_dev, t, pdMS_TO_TICKS(2500)) != ESP_OK)
    {
		ESP_LOGE(TAG, "spi_device_queue_trans error!");
    }
}

static void lcd_100ask_pre_cb(spi_transaction_t *transaction) {
	gpio_set_level(LCD_100ASK_PIN_DC, (int)transaction->user & 1);
}

static void lcd_100ask_send_cmd(const lcd_display_command_t *command){
	spi_queue_transaction(&command->command, 1, 0);
    if (command->data && command->data_size > 0)
        spi_queue_transaction(command->data, command->data_size, 1);
	
    // Wait the required time
	if (command->wait_ms > 0) {
		vTaskDelay(command->wait_ms / portTICK_PERIOD_MS);
	}
}

static void lcd_100ask_send_multi( const lcd_display_command_t *sequence){
    while (sequence->command != LCD_DISPLAY_CMDLIST_END) {
		lcd_100ask_send_cmd(sequence);
		sequence++;
	}
}



/* Display config */
static void lcd_100ask_config(lcd_display_rotation_t rotation){
// 170X320
#if CONFIG_USE_100ASK_DISPLAY_SCREEN_170X320
	const lcd_display_command_t init_sequence[] = {
		{0x36, 120, 1, (const uint8_t *)"\x00"},
		{0x3A, 0, 1, (const uint8_t *)"\x05"},
		{0xB2, 0, 5, (const uint8_t *)"\x0C\x0C\x00\x33\x33"},
		{0xB7, 0, 1, (const uint8_t *)"\x35"},
		{0xBB, 0, 1, (const uint8_t *)"\x1A"},
		{0xC0, 0, 1, (const uint8_t *)"\x2C"},
		{0xC2, 0, 1, (const uint8_t *)"\x01"},
		{0xC3, 0, 1, (const uint8_t *)"\x0B"},
		{0xC4, 0, 1, (const uint8_t *)"\x20"},
		{0xC6, 0, 1, (const uint8_t *)"\x0F"},
		{0xD0, 0, 2, (const uint8_t *)"\xA4\xA1"},
		{0xE0, 0, 14, (const uint8_t *)"\x00\x03\x07\x08\x07\x15\x2A\x44\x42\x0A\x17\x18\x25\x27"},
		{0XE1, 0, 14, (const uint8_t *)"\x00\x03\x08\x07\x07\x23\x2A\x43\x42\x09\x18\x17\x25\x27"},
		{0x21, 0, 0, (const uint8_t *)"\x0"},
		{0x11, 120, 0, (const uint8_t *)"\x0"},
		{0x29, 0, 0, (const uint8_t *)"\x0"},
		{0xff, 0, 0, NULL}, // End of commands
	};
	lcd_100ask_send_multi(init_sequence);

	if(rotation == LCD_DISPLAY_ROTATION_90)
	{
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 35;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x60"}, // 90  (1<<6)|(1<<5) //BGR==1,MY==1,MX==0,MV==1
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else if(rotation == LCD_DISPLAY_ROTATION_180)
	{
		lcd_dev_xoffset = 35;
		lcd_dev_yoffset = 0;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\xc0"}, // 180 (1<<6)|(1<<7) //BGR==1,MY==0,MX==0,MV==0
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else if(rotation == LCD_DISPLAY_ROTATION_270)
	{
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 35;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\xa0"}, // 270 (1<<7)|(1<<5) //BGR==1,MY==1,MX==0,MV==1
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else
	{
		lcd_dev_xoffset = 35;
		lcd_dev_yoffset = 0;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x0"}, // 0 (0) //BGR==1,MY==0,MX==0,MV==0
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}

	ESP_LOGI(TAG, "Successfully initialized 170x320!");

//240X240
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_240X240
	const lcd_display_command_t init_sequence[] = {
		{0x36, 120, 1, (const uint8_t *)"\x00"},
		{0x3A, 0, 1, (const uint8_t *)"\x05"},
		{0xB2, 0, 5, (const uint8_t *)"\x0C\x0C\x00\x33\x33"},
		{0xB7, 0, 1, (const uint8_t *)"\x35"},
		{0xBB, 0, 1, (const uint8_t *)"\x19"},
		{0xC0, 0, 1, (const uint8_t *)"\x2C"},
		{0xC2, 0, 1, (const uint8_t *)"\x01"},
		{0xC3, 0, 1, (const uint8_t *)"\x12"},
		{0xC4, 0, 1, (const uint8_t *)"\x20"},
		{0xC6, 0, 1, (const uint8_t *)"\x0F"},
		{0xD0, 0, 2, (const uint8_t *)"\xA4\xA1"},
		{0xE0, 0, 14, (const uint8_t *)"\xD0\x04\x0D\x11\x13\x2B\x3F\x54\x4C\x18\x0D\x0B\x1F\x23"},
		{0XE1, 0, 14, (const uint8_t *)"\xD0\x04\x0C\x11\x13\x2C\x3F\x44\x51\x2F\x1F\x1F\x20\x23"},
		{0x21, 0, 0, (const uint8_t *)"\x0"},
		{0x11, 120, 0, (const uint8_t *)"\x0"},
		{0x29, 0, 0, (const uint8_t *)"\x0"},
		{0xff, 0, 0, NULL}, // End of commands
	};
	lcd_100ask_send_multi(init_sequence);

	if(rotation == LCD_DISPLAY_ROTATION_90)
	{
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 0;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x60"}, // 90  (1<<6)|(1<<5) //BGR==1,MY==1,MX==0,MV==1
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else if(rotation == LCD_DISPLAY_ROTATION_180)
	{
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 80;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\xc0"}, // 180 (1<<6)|(1<<7) //BGR==1,MY==0,MX==0,MV==0
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else if(rotation == LCD_DISPLAY_ROTATION_270)
	{
		lcd_dev_xoffset = 80;
		lcd_dev_yoffset = 0;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\xa0"}, // 270 (1<<7)|(1<<5) //BGR==1,MY==1,MX==0,MV==1
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else
	{
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 0;
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x0"}, // 0 (0) //BGR==1,MY==0,MX==0,MV==0
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}

	ESP_LOGI(TAG, "Successfully initialized 240x240!");

//320X480
#elif CONFIG_USE_100ASK_DISPLAY_SCREEN_320X480
const lcd_display_command_t init_sequence[] = {
		{0x11, 120, 1, (const uint8_t *)"\x00"},
		{0xF0, 0, 1, (const uint8_t *)"\xC3"},
		{0xF0, 0, 1, (const uint8_t *)"\x96"},
		{0x36, 0, 1, (const uint8_t *)"\x48"},
		{0xB4, 0, 1, (const uint8_t *)"\x01"},
		{0xB7, 0, 1, (const uint8_t *)"\xC6"},
		{0xE8, 0, 8, (const uint8_t *)"\x40\x8A\x00\x00\x29\x19\xA5\x33"},
		{0xC1, 0, 1, (const uint8_t *)"\x06"},
		{0xC2, 0, 1, (const uint8_t *)"\xA7"},
		{0xC5, 0, 1, (const uint8_t *)"\x18"},
		{0xE0, 0, 14, (const uint8_t *)"\xF0\x09\x0B\x06\x04\x15\x2F\x54\x42\x3C\x17\x14\x18\x1B"},
		{0XE1, 0, 14, (const uint8_t *)"\xF0\x09\x0B\x06\x04\x03\x2D\x43\x42\x3B\x16\x14\x17\x1B"},
		{0xF0, 0, 1, (const uint8_t *)"\x3C"},
		{0xF0, 0, 1, (const uint8_t *)"\x69"},
		{0x3A, 120, 1, (const uint8_t *)"\x55"},
		{0x29, 0, 0, (const uint8_t *)"\x0"},
		{0xff, 0, 0, NULL}, // End of commands
	};
	lcd_100ask_send_multi(init_sequence);

	if(rotation == LCD_DISPLAY_ROTATION_90)
	{
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\xE8"}, // 90  (((1<<7)|(1<<6)|(1<<5))|0X08) D2U_R2L
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else if(rotation == LCD_DISPLAY_ROTATION_180)
	{
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x88"}, // 180 (((1<<7)|(0<<6)|(0<<5))|0X08) L2R_D2U
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else if(rotation == LCD_DISPLAY_ROTATION_270)
	{
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x28"}, // 270 (((0<<7)|(0<<6)|(1<<5))|0X08) U2D_L2R
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}
	else
	{
		const lcd_display_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x48"}, // 0 (((0<<7)|(1<<6)|(0<<5))|0X08) R2L_U2D
			{0xff, 0, 0, NULL},                    // End of commands
		};
		lcd_100ask_send_multi(set_rotation);
	}

	const lcd_display_command_t set_screen_size[] = {
		{0x2A, 0, 4, (const uint8_t *)"\x00\x00\x01\x3f"},
		{0x2B, 0, 4, (const uint8_t *)"\x00\x00\x01\xdf"},
		{0xff, 0, 0, NULL},                             // End of commands
	};
	lcd_100ask_send_multi(set_screen_size);

	ESP_LOGI(TAG, "Successfully initialized 320x480!");
#else
	#error "Display screen init sequence is not defined for this device!"
#endif
}

#endif /* CONFIG_USE_100ASK_SPI_DISPLAY_SCREEN && CONFIG_USE_100ASK_DISPLAY_SCREEN_SPI_DRIVE */
