/**
 * @file tft_lcd_170x320.c
 *
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

#include "driver/gpio.h"

#include "esp_log.h"

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_170X320

#include "tft_lcd_100ask_hal.h"
#include "tft_lcd_170x320.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "170X320_driver"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void TFT_LCD_170X320_reset(void);
static void TFT_LCD_170X320_send_cmd(TFT_LCD_170X320_driver_t *driver, const TFT_LCD_170X320_command_t *command);
static void TFT_LCD_170X320_config(TFT_LCD_170X320_driver_t *driver, TFT_LCD_170X320_rotation_t rotation);
static void TFT_LCD_170X320_pre_cb(spi_transaction_t *transaction);
static void TFT_LCD_170X320_queue_empty(TFT_LCD_170X320_driver_t *driver);
static void TFT_LCD_170X320_multi_cmd(TFT_LCD_170X320_driver_t *driver, const TFT_LCD_170X320_command_t *sequence);

/**********************
 *  STATIC VARIABLES
 **********************/
static uint16_t lcd_dev_xoffset = 0;
static uint16_t lcd_dev_yoffset = 0;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
bool TFT_LCD_170X320_init(TFT_LCD_170X320_driver_t *driver){

    //Allocate the buffer memory
    driver->buffer = (TFT_LCD_170X320_color_t *)heap_caps_malloc(driver->buffer_size * 2 * sizeof(TFT_LCD_170X320_color_t), MALLOC_CAP_DMA);
	if (driver->buffer == NULL)
		driver->buffer = (TFT_LCD_170X320_color_t *)malloc(driver->buffer_size * 2 * sizeof(TFT_LCD_170X320_color_t));
    assert(driver->buffer != NULL);

    ESP_LOGI(TAG,"Display buffer allocated with a size of: %i",driver->buffer_size * 2 * sizeof(TFT_LCD_170X320_color_t));

    //Set-up the display buffers
    driver->buffer_primary =  driver->buffer;
    driver->buffer_secondary = driver->buffer + driver->buffer_size;
    driver->current_buffer = driver->buffer_primary;
    driver->queue_fill = 0;

    driver->data.driver = driver;
	driver->data.data = true;
	driver->command.driver = driver;
	driver->command.data = false;

    ESP_LOGI(TAG,"Set RST pin: %i \n Set DC pin: %i", SPI_TFT_LCD_100ASK_DISP_PIN_RST, SPI_TFT_LCD_100ASK_DISP_PIN_DC);

    // Set-Up SPI BUS
    spi_bus_config_t buscfg = {
		.mosi_io_num    = SPI_TFT_LCD_100ASK_DISP_PIN_MOSI,
		.miso_io_num    = -1,
		.sclk_io_num    = SPI_TFT_LCD_100ASK_DISP_PIN_CLK,
		.quadwp_io_num  = -1,
		.quadhd_io_num  = -1,
		.max_transfer_sz= driver->buffer_size * 2 * sizeof(TFT_LCD_170X320_color_t), // 2 buffers with 2 bytes for pixel
	};

    // Configure SPI BUS
    spi_device_interface_config_t devcfg = {
		.clock_speed_hz = HSPI_CLK_SPEED,
		.mode           = SPI_TFT_LCD_100ASK_DISP_SPI_MODE,
		.spics_io_num   = SPI_TFT_LCD_100ASK_DISP_PIN_CS,
		.queue_size     = SPI_TFT_LCD_100ASK_QUEUE_SIZE,
		.pre_cb         = TFT_LCD_170X320_pre_cb,
		.flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX,	/* dummy bits should be explicitly handled via DISP_SPI_VARIABLE_DUMMY as needed */
	};

	ESP_LOGI(TAG, "Initializing SPI bus...");
    if(spi_bus_initialize(SPI2_HOST, &buscfg, (spi_dma_chan_t)SPI_DMA_CH_AUTO) != ESP_OK){
        ESP_LOGE(TAG,"SPI Bus initialization failed.");
		free(driver->buffer);
        return false;
    }

    if(spi_bus_add_device(SPI2_HOST, &devcfg, &driver->spi) != ESP_OK){
        ESP_LOGE(TAG,"SPI Bus add device failed.");
		free(driver->buffer);
        return false;
    }

    ESP_LOGI(TAG,"SPI Bus configured correctly.");

    // Set the RESET and DC PIN
    gpio_pad_select_gpio(SPI_TFT_LCD_100ASK_DISP_PIN_RST);
    gpio_pad_select_gpio(SPI_TFT_LCD_100ASK_DISP_PIN_DC);
    gpio_set_direction(SPI_TFT_LCD_100ASK_DISP_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_TFT_LCD_100ASK_DISP_PIN_DC, GPIO_MODE_OUTPUT);

    // Set the screen configuration
    TFT_LCD_170X320_reset();

#if   CONFIG_TFT_LCD_100ASK_DISP_ROTATION == 90
	TFT_LCD_170X320_config(driver, TFT_LCD_170X320_ROTATION_90);
#elif CONFIG_TFT_LCD_100ASK_DISP_ROTATION == 180
	TFT_LCD_170X320_config(driver, TFT_LCD_170X320_ROTATION_180);
#elif CONFIG_TFT_LCD_100ASK_DISP_ROTATION == 270
	TFT_LCD_170X320_config(driver, TFT_LCD_170X320_ROTATION_270);
#else
	TFT_LCD_170X320_config(driver, TFT_LCD_170X320_ROTATION_0);
#endif
    

    ESP_LOGI(TAG,"Display configured and ready to work.");

    return true;
}

void TFT_LCD_170X320_fill_area(TFT_LCD_170X320_driver_t *driver, TFT_LCD_170X320_color_t color, uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height){
    // Fill the buffer with the selected color
	for (size_t i = 0; i < driver->buffer_size * 2; ++i) {
		driver->buffer[i] = color;
	}

    // Set the working area on the screen
	//TFT_LCD_170X320_set_window(driver, start_x, start_y, start_x + width - 1, start_y + height - 1);
	TFT_LCD_170X320_set_window(driver, start_x, start_y, start_x + width, start_y + height);

	size_t bytes_to_write = width * height * 2;
	size_t transfer_size = driver->buffer_size * 2 * sizeof(TFT_LCD_170X320_color_t);

	spi_transaction_t trans;
    spi_transaction_t *rtrans;

	memset(&trans, 0, sizeof(trans));
	trans.tx_buffer = driver->buffer;
	trans.user = &driver->data;
	trans.length = transfer_size * 8;
	trans.rxlength = 0;

	
	while (bytes_to_write > 0) {
		if (driver->queue_fill >= SPI_TFT_LCD_100ASK_QUEUE_SIZE) {
			spi_device_get_trans_result(driver->spi, &rtrans, portMAX_DELAY);
			driver->queue_fill--;
		}
		if (bytes_to_write < transfer_size) {
			transfer_size = bytes_to_write;
		}
		spi_device_queue_trans(driver->spi, &trans, portMAX_DELAY);
		driver->queue_fill++;
		bytes_to_write -= transfer_size;
	}

	TFT_LCD_170X320_queue_empty(driver);
}

void TFT_LCD_170X320_write_pixels(TFT_LCD_170X320_driver_t *driver, TFT_LCD_170X320_color_t *pixels, size_t length){
	TFT_LCD_170X320_queue_empty(driver);

	spi_transaction_t *trans = driver->current_buffer == driver->buffer_primary ? &driver->trans_a : &driver->trans_b;
	memset(trans, 0, sizeof(&trans));
	trans->tx_buffer = driver->current_buffer;
	trans->user = &driver->data;
	trans->length = length * sizeof(TFT_LCD_170X320_color_t) * 8;
	trans->rxlength = 0;

	spi_device_queue_trans(driver->spi, trans, portMAX_DELAY);
	driver->queue_fill++;
}

void TFT_LCD_170X320_write_lines(TFT_LCD_170X320_driver_t *driver, int ypos, int xpos, int width, uint16_t *linedata, int lineCount){
	// TFT_LCD_170X320_set_window(driver,xpos,ypos,240,ypos +20);
    //int size = width * 2 * 8 * lineCount;

    //driver->buffer_secondary = linedata;
    //driver->current_buffer = driver->buffer_secondary;


    //TFT_LCD_170X320_write_pixels(driver, driver->buffer_primary, size);
    driver->buffer_size = SCR_WIDTH*20; 
    TFT_LCD_170X320_set_window(driver,0,ypos,SCR_WIDTH,ypos +20);
    // TFT_LCD_170X320_write_pixels(driver, driver->current_buffer, driver->buffer_size);
    TFT_LCD_170X320_swap_buffers(driver);
}

void TFT_LCD_170X320_swap_buffers(TFT_LCD_170X320_driver_t *driver){
	TFT_LCD_170X320_write_pixels(driver, driver->current_buffer, driver->buffer_size);
	driver->current_buffer = driver->current_buffer == driver->buffer_primary ? driver->buffer_secondary : driver->buffer_primary;
}

void TFT_LCD_170X320_set_window(TFT_LCD_170X320_driver_t *driver, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y){
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

	TFT_LCD_170X320_command_t sequence[] = {
		{TFT_LCD_170X320_CMD_CASET, 0, 4, caset},
		{TFT_LCD_170X320_CMD_RASET, 0, 4, raset},
		{TFT_LCD_170X320_CMD_RAMWR, 0, 0, NULL},
		{TFT_LCD_170X320_CMDLIST_END, 0, 0, NULL},
	};

	TFT_LCD_170X320_multi_cmd(driver, sequence);
}

void TFT_LCD_170X320_set_endian(TFT_LCD_170X320_driver_t *driver){
	const TFT_LCD_170X320_command_t init_sequence2[] = {
		{TFT_LCD_170X320_CMD_RAMCTRL, 0, 2, (const uint8_t *)"\x00\xc0"},
		{TFT_LCD_170X320_CMDLIST_END, 0, 0, NULL}, 
	};

	TFT_LCD_170X320_multi_cmd(driver, init_sequence2);
}

void TFT_LCD_170X320_set_rotation(TFT_LCD_170X320_driver_t *driver, TFT_LCD_170X320_rotation_t rotation)
{
	TFT_LCD_170X320_reset();
    TFT_LCD_170X320_config(driver, rotation);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void TFT_LCD_170X320_reset(void) {
	gpio_set_level(SPI_TFT_LCD_100ASK_DISP_PIN_RST, 0);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	gpio_set_level(SPI_TFT_LCD_100ASK_DISP_PIN_RST, 1);
	vTaskDelay(20 / portTICK_PERIOD_MS);
}

static void TFT_LCD_170X320_pre_cb(spi_transaction_t *transaction) {
	const TFT_LCD_170X320_transaction_data_t *data = (TFT_LCD_170X320_transaction_data_t *)transaction->user;
	gpio_set_level(SPI_TFT_LCD_100ASK_DISP_PIN_DC, data->data);
}

static void TFT_LCD_170X320_config(TFT_LCD_170X320_driver_t *driver, TFT_LCD_170X320_rotation_t rotation){
	const TFT_LCD_170X320_command_t init_sequence[] = {
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
	TFT_LCD_170X320_multi_cmd(driver, init_sequence);

	if(rotation == TFT_LCD_170X320_ROTATION_90)
	{
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 35;
		const TFT_LCD_170X320_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x60"}, // 90  (1<<6)|(1<<5) //BGR==1,MY==1,MX==0,MV==1
			{0xff, 0, 0, NULL},                    // End of commands
		};
		TFT_LCD_170X320_multi_cmd(driver, set_rotation);
	}
	else if(rotation == TFT_LCD_170X320_ROTATION_180)
	{
		lcd_dev_xoffset = 35;
		lcd_dev_yoffset = 0;
		const TFT_LCD_170X320_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\xc0"}, // 180 (1<<6)|(1<<7) //BGR==1,MY==0,MX==0,MV==0
			{0xff, 0, 0, NULL},                    // End of commands
		};
		TFT_LCD_170X320_multi_cmd(driver, set_rotation);
	}
	else if(rotation == TFT_LCD_170X320_ROTATION_270)
	{
		lcd_dev_xoffset = 0;
		lcd_dev_yoffset = 35;
		const TFT_LCD_170X320_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\xa0"}, // 270 (1<<7)|(1<<5) //BGR==1,MY==1,MX==0,MV==1
			{0xff, 0, 0, NULL},                    // End of commands
		};
		TFT_LCD_170X320_multi_cmd(driver, set_rotation);
	}
	else
	{
		lcd_dev_xoffset = 35;
		lcd_dev_yoffset = 0;
		const TFT_LCD_170X320_command_t set_rotation[] = {
			{0x36, 0, 1, (const uint8_t *)"\x0"}, // 0 (0) //BGR==1,MY==0,MX==0,MV==0
			{0xff, 0, 0, NULL},                    // End of commands
		};
		TFT_LCD_170X320_multi_cmd(driver, set_rotation);
	}
}


static void TFT_LCD_170X320_send_cmd(TFT_LCD_170X320_driver_t *driver, const TFT_LCD_170X320_command_t *command){
    spi_transaction_t *return_trans;
	spi_transaction_t data_trans;
    
    // Check if the SPI queue is empty
    TFT_LCD_170X320_queue_empty(driver);

    // Send the command
	memset(&data_trans, 0, sizeof(data_trans));
	data_trans.length = 8; // 8 bits
	data_trans.tx_buffer = &command->command;
	data_trans.user = &driver->command;

	spi_device_queue_trans(driver->spi, &data_trans, portMAX_DELAY);
	spi_device_get_trans_result(driver->spi, &return_trans, portMAX_DELAY);

    // Send the data if the command has.
	if (command->data_size > 0) {
		memset(&data_trans, 0, sizeof(data_trans));
		data_trans.length = command->data_size * 8;
		data_trans.tx_buffer = command->data;
		data_trans.user = &driver->data;

		spi_device_queue_trans(driver->spi, &data_trans, portMAX_DELAY);
		spi_device_get_trans_result(driver->spi, &return_trans, portMAX_DELAY);
	}

    // Wait the required time
	if (command->wait_ms > 0) {
		vTaskDelay(command->wait_ms / portTICK_PERIOD_MS);
	}
}

static void TFT_LCD_170X320_multi_cmd(TFT_LCD_170X320_driver_t *driver, const TFT_LCD_170X320_command_t *sequence){
    while (sequence->command != TFT_LCD_170X320_CMDLIST_END) {
		TFT_LCD_170X320_send_cmd(driver, sequence);
		sequence++;
	}
}

static void TFT_LCD_170X320_queue_empty(TFT_LCD_170X320_driver_t *driver){
	spi_transaction_t *return_trans;

	while (driver->queue_fill > 0) {
		spi_device_get_trans_result(driver->spi, &return_trans, portMAX_DELAY);
		driver->queue_fill--;
	}
}


#endif
