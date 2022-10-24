/**
 * @file st7796s.c
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

#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_ST7796S

#include "tft_lcd_100ask_drivers.h"
#include "st7796s.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "ST7796S_driver"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ST7796S_send_cmd(ST7796S_driver_t *driver, const ST7796S_command_t *command);
static void ST7796S_config(ST7796S_driver_t *driver);
static void ST7796S_pre_cb(spi_transaction_t *transaction);
static void ST7796S_queue_empty(ST7796S_driver_t *driver);
static void ST7796S_multi_cmd(ST7796S_driver_t *driver, const ST7796S_command_t *sequence);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
bool ST7796S_init(ST7796S_driver_t *driver){

    //Allocate the buffer memory
    driver->buffer = (ST7796S_color_t *)heap_caps_malloc(driver->buffer_size * 2 * sizeof(ST7796S_color_t), MALLOC_CAP_DMA);
	if (driver->buffer == NULL)
		driver->buffer = (ST7796S_color_t *)malloc(driver->buffer_size * 2 * sizeof(ST7796S_color_t));
    assert(driver->buffer != NULL);

    ESP_LOGI(TAG,"Display buffer allocated with a size of: %i",driver->buffer_size * 2 * sizeof(ST7796S_color_t));

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
		.max_transfer_sz= driver->buffer_size * 2 * sizeof(ST7796S_color_t), // 2 buffers with 2 bytes for pixel
	};

    // Configure SPI BUS
    spi_device_interface_config_t devcfg = {
		.clock_speed_hz = HSPI_CLK_SPEED,
		.mode           = SPI_TFT_LCD_100ASK_DISP_SPI_MODE,
		.spics_io_num   = SPI_TFT_LCD_100ASK_DISP_PIN_CS,
		.queue_size     = ST7796S_SPI_QUEUE_SIZE,
		.pre_cb         = ST7796S_pre_cb,
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
    ST7796S_reset(driver);
    ST7796S_config(driver);

    ESP_LOGI(TAG,"Display configured and ready to work.");

    return true;
}


void ST7796S_reset(ST7796S_driver_t *driver) {
	gpio_set_level(SPI_TFT_LCD_100ASK_DISP_PIN_RST, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(SPI_TFT_LCD_100ASK_DISP_PIN_RST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}


void ST7796S_fill_area(ST7796S_driver_t *driver, ST7796S_color_t color, uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height){
    // Fill the buffer with the selected color
	for (size_t i = 0; i < driver->buffer_size * 2; ++i) {
		driver->buffer[i] = color;
	}

    // Set the working area on the screen
	//ST7796S_set_window(driver, start_x, start_y, start_x + width - 1, start_y + height - 1);
	ST7796S_set_window(driver, start_x, start_y, start_x + width, start_y + height);

	size_t bytes_to_write = width * height * 2;
	size_t transfer_size = driver->buffer_size * 2 * sizeof(ST7796S_color_t);

	spi_transaction_t trans;
    spi_transaction_t *rtrans;

	memset(&trans, 0, sizeof(trans));
	trans.tx_buffer = driver->buffer;
	trans.user = &driver->data;
	trans.length = transfer_size * 8;
	trans.rxlength = 0;

	
	while (bytes_to_write > 0) {
		if (driver->queue_fill >= ST7796S_SPI_QUEUE_SIZE) {
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

	ST7796S_queue_empty(driver);
}

void ST7796S_write_pixels(ST7796S_driver_t *driver, ST7796S_color_t *pixels, size_t length){
	ST7796S_queue_empty(driver);

	spi_transaction_t *trans = driver->current_buffer == driver->buffer_primary ? &driver->trans_a : &driver->trans_b;
	memset(trans, 0, sizeof(&trans));
	trans->tx_buffer = driver->current_buffer;
	trans->user = &driver->data;
	trans->length = length * sizeof(ST7796S_color_t) * 8;
	trans->rxlength = 0;

	spi_device_queue_trans(driver->spi, trans, portMAX_DELAY);
	driver->queue_fill++;
}

void ST7796S_write_lines(ST7796S_driver_t *driver, int ypos, int xpos, int width, uint16_t *linedata, int lineCount){
	// ST7796S_set_window(driver,xpos,ypos,240,ypos +20);
    //int size = width * 2 * 8 * lineCount;

    //driver->buffer_secondary = linedata;
    //driver->current_buffer = driver->buffer_secondary;


    //ST7796S_write_pixels(driver, driver->buffer_primary, size);
    driver->buffer_size = SCR_WIDTH*20; 
    ST7796S_set_window(driver,0,ypos,SCR_WIDTH,ypos +20);
    // ST7796S_write_pixels(driver, driver->current_buffer, driver->buffer_size);
    ST7796S_swap_buffers(driver);
}

void ST7796S_swap_buffers(ST7796S_driver_t *driver){
	ST7796S_write_pixels(driver, driver->current_buffer, driver->buffer_size);
	driver->current_buffer = driver->current_buffer == driver->buffer_primary ? driver->buffer_secondary : driver->buffer_primary;
}

void ST7796S_set_window(ST7796S_driver_t *driver, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y){
	uint8_t caset[4];
	uint8_t raset[4];
    
	caset[0] = (uint8_t)(start_x >> 8) & 0xFF;
	caset[1] = (uint8_t)(start_x & 0xff);
	caset[2] = (uint8_t)(end_x >> 8) & 0xFF;
	caset[3] = (uint8_t)(end_x & 0xff) ;
	
	raset[0] = (uint8_t)(start_y >> 8) & 0xFF;
	raset[1] = (uint8_t)(start_y & 0xff);
	raset[2] = (uint8_t)(end_y >> 8) & 0xFF;
	raset[3] = (uint8_t)(end_y & 0xff);

	ST7796S_command_t sequence[] = {
		{ST7796S_CMD_CASET, 0, 4, caset},
		{ST7796S_CMD_RASET, 0, 4, raset},
		{ST7796S_CMD_RAMWR, 0, 0, NULL},
		{ST7796S_CMDLIST_END, 0, 0, NULL},
	};

	ST7796S_multi_cmd(driver, sequence);
}

void ST7796S_set_endian(ST7796S_driver_t *driver){
	const ST7796S_command_t init_sequence2[] = {
		{ST7796S_CMD_RAMCTRL, 0, 2, (const uint8_t *)"\x00\xc0"},
		{ST7796S_CMDLIST_END, 0, 0, NULL}, 
	};

	ST7796S_multi_cmd(driver, init_sequence2);
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void ST7796S_pre_cb(spi_transaction_t *transaction) {
	const ST7796S_transaction_data_t *data = (ST7796S_transaction_data_t *)transaction->user;
	gpio_set_level(SPI_TFT_LCD_100ASK_DISP_PIN_DC, data->data);
}

static void ST7796S_config(ST7796S_driver_t *driver){
	const ST7796S_command_t init_sequence[] = {
		{0xCF, 0, 3, (const uint8_t *)"\x00\x83\x30"},
		{0xED, 0, 4, (const uint8_t *)"\x64\x03\x12\x81"},
		{0xE8, 0, 3, (const uint8_t *)"\x85\x01\x79"},
		{0xCB, 0, 5, (const uint8_t *)"\x39\x2C\x00\x34\x02"},
		{0xF7, 0, 1, (const uint8_t *)"\x20"},
		{0xEA, 0, 2, (const uint8_t *)"\x00\x00"},
		{0xC0, 0, 1, (const uint8_t *)"\x26"},		 /*Power control*/
		{0xC1, 0, 1, (const uint8_t *)"\x11"},		 /*Power control */
		{0xC5, 0, 2, (const uint8_t *)"\x35\x3E"}, /*VCOM control*/
		{0xC7, 0, 1, (const uint8_t *)"\xBE"},		 /*VCOM control*/
		{0x36, 0, 1, (const uint8_t *)"\x28"},		 /*Memory Access Control*/
		{0x3A, 0, 1, (const uint8_t *)"\x55"},		 /*Pixel Format Set*/
		{0xB1, 0, 2, (const uint8_t *)"\x00\x1B"},
		{0xF2, 0, 1, (const uint8_t *)"\x08"},
		{0x26, 0, 1, (const uint8_t *)"\x01"},
		{0xE0, 0, 15, (const uint8_t *)"\x1F\x1A\x18\x0A\x0F\x06\x45\x87\x32\x0A\x07\x02\x07\x05\x00"},
		{0XE1, 0, 15, (const uint8_t *)"\x00\x25\x27\x05\x10\x09\x3A\x78\x4D\x05\x18\x0D\x38\x3A\x1F"},
		{0x2A, 0, 4, (const uint8_t *)"\x00\x00\x00\xEF"},
		{0x2B, 0, 4, (const uint8_t *)"\x00\x00\x01\x3f"},
		{0x2C, 0, 0, (const uint8_t *)"\x0"},
		{0xB7, 0, 1, (const uint8_t *)"\x07"},
		{0xB6, 0, 4, (const uint8_t *)"\x0A\x82\x27\x00"},
		{0x11, 100, 0, (const uint8_t *)"\x0"},
		{0x29, 100, 0, (const uint8_t *)"\x0"},
		{0xff, 0, 0, NULL},
	};
	ST7796S_multi_cmd(driver, init_sequence);

	const ST7796S_command_t set_orientation[] = {
		{0x36, 0, 4, (const uint8_t *)"\x48\x88\x28\xE8}"},
		{0xff, 0, 0, NULL},                  // End of commands
	};
	ST7796S_multi_cmd(driver, set_orientation);

	const ST7796S_command_t set_invert_colors[] = {
		{0x20, 0, 0, NULL},
		{0xff, 0, 0, NULL},                  // End of commands
	};
	ST7796S_multi_cmd(driver, set_invert_colors);
}


static void ST7796S_send_cmd(ST7796S_driver_t *driver, const ST7796S_command_t *command){
    spi_transaction_t *return_trans;
	spi_transaction_t data_trans;
    
    // Check if the SPI queue is empty
    ST7796S_queue_empty(driver);

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

static void ST7796S_multi_cmd(ST7796S_driver_t *driver, const ST7796S_command_t *sequence){
    while (sequence->command != ST7796S_CMDLIST_END) {
		ST7796S_send_cmd(driver, sequence);
		sequence++;
	}
}

static void ST7796S_queue_empty(ST7796S_driver_t *driver){
	spi_transaction_t *return_trans;

	while (driver->queue_fill > 0) {
		spi_device_get_trans_result(driver->spi, &return_trans, portMAX_DELAY);
		driver->queue_fill--;
	}
}


#endif
