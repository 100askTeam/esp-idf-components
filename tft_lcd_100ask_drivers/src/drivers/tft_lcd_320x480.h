/**
 * @file tft_lcd_320x480.h
 *
 */

#ifndef TFT_LCD_320X480_H
#define TFT_LCD_320X480_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_SPI_TFT_LCD_320X480

#include "driver/spi_master.h"

/*********************
 *      DEFINES
 *********************/

/*******************************
 *      TFT_LCD_320X480 Commands
 * *****************************/

// System Function Command Table 1
#define TFT_LCD_320X480_CMD_NOP               0x00 // No operation
#define TFT_LCD_320X480_CMD_SWRESET           0x01 // Software reset
#define TFT_LCD_320X480_CMD_RDDID             0x04 // Read display ID
#define TFT_LCD_320X480_CMD_RDDST             0x09 // Read display status
#define TFT_LCD_320X480_CMD_RDDPM             0x0a // Read display power
#define TFT_LCD_320X480_CMD_RDDMADCTL         0x0b // Read display
#define TFT_LCD_320X480_CMD_RDDCOLMOD         0x0c // Read display pixel
#define TFT_LCD_320X480_CMD_RDDIM             0x0d // Read display image
#define TFT_LCD_320X480_CMD_RDDSM             0x0e // Read display signal
#define TFT_LCD_320X480_CMD_RDDSDR            0x0f // Read display self-diagnostic result
#define TFT_LCD_320X480_CMD_SLPIN             0x10 // Sleep in
#define TFT_LCD_320X480_CMD_SLPOUT            0x11 // Sleep out
#define TFT_LCD_320X480_CMD_PTLON             0x12 // Partial mode on
#define TFT_LCD_320X480_CMD_NORON             0x13 // Partial off (Normal)
#define TFT_LCD_320X480_CMD_INVOFF            0x20 // Display inversion off
#define TFT_LCD_320X480_CMD_INVON             0x21 // Display inversion on
#define TFT_LCD_320X480_CMD_GAMSET            0x26 // Gamma set
#define TFT_LCD_320X480_CMD_DISPOFF           0x28 // Display off
#define TFT_LCD_320X480_CMD_DISPON            0x29 // Display on
#define TFT_LCD_320X480_CMD_CASET             0x2a // Column address set
#define TFT_LCD_320X480_CMD_RASET             0x2b // Row address set
#define TFT_LCD_320X480_CMD_RAMWR             0x2c // Memory write
#define TFT_LCD_320X480_CMD_RAMRD             0x2e // Memory read
#define TFT_LCD_320X480_CMD_PTLAR             0x30 // Partial start/end address set
#define TFT_LCD_320X480_CMD_VSCRDEF           0x33 // Vertical scrolling definition
#define TFT_LCD_320X480_CMD_TEOFF             0x34 // Tearing line effect off
#define TFT_LCD_320X480_CMD_TEON              0x35 // Tearing line effect on
#define TFT_LCD_320X480_CMD_MADCTL            0x36 // Memory data access control
#define TFT_LCD_320X480_CMD_VSCRSADD          0x37 // Vertical address scrolling
#define TFT_LCD_320X480_CMD_IDMOFF            0x38 // Idle mode off
#define TFT_LCD_320X480_CMD_IDMON             0x39 // Idle mode on
#define TFT_LCD_320X480_CMD_COLMOD            0x3a // Interface pixel format
#define TFT_LCD_320X480_CMD_RAMWRC            0x3c // Memory write continue
#define TFT_LCD_320X480_CMD_RAMRDC            0x3e // Memory read continue
#define TFT_LCD_320X480_CMD_TESCAN            0x44 // Set tear scanline
#define TFT_LCD_320X480_CMD_RDTESCAN          0x45 // Get scanline
#define TFT_LCD_320X480_CMD_WRDISBV           0x51 // Write display brightness
#define TFT_LCD_320X480_CMD_RDDISBV           0x52 // Read display brightness value
#define TFT_LCD_320X480_CMD_WRCTRLD           0x53 // Write CTRL display
#define TFT_LCD_320X480_CMD_RDCTRLD           0x54 // Read CTRL value display
#define TFT_LCD_320X480_CMD_WRCACE            0x55 // Write content adaptive brightness control and Color enhancemnet
#define TFT_LCD_320X480_CMD_RDCABC            0x56 // Read content adaptive brightness control
#define TFT_LCD_320X480_CMD_WRCABCMB          0x5e // Write CABC minimum brightness
#define TFT_LCD_320X480_CMD_RDCABCMB          0x5f // Read CABC minimum brightness
#define TFT_LCD_320X480_CMD_RDABCSDR          0x68 // Read Automatic Brightness Control Self-Diagnostic Result
#define TFT_LCD_320X480_CMD_RDID1             0xda // Read ID1
#define TFT_LCD_320X480_CMD_RDID2             0xdb // Read ID2
#define TFT_LCD_320X480_CMD_RDID3             0xdc // Read ID3

// System Function Command Table 2
#define TFT_LCD_320X480_CMD_RAMCTRL           0xb0 // RAM Control
#define TFT_LCD_320X480_CMD_RGBCTRL           0xb1 // RGB Control
#define TFT_LCD_320X480_CMD_PORCTRL           0xb2 // Porch control
#define TFT_LCD_320X480_CMD_FRCTRL1           0xb3 // Frame Rate Control 1
#define TFT_LCD_320X480_CMD_GCTRL             0xb7 // Gate control
#define TFT_LCD_320X480_CMD_DGMEN             0xba // Digital Gamma Enable
#define TFT_LCD_320X480_CMD_VCOMS             0xbb // VCOM Setting
#define TFT_LCD_320X480_CMD_LCMCTRL           0xc0 // LCM Control
#define TFT_LCD_320X480_CMD_IDSET             0xc1 // ID Setting
#define TFT_LCD_320X480_CMD_VDVVRHEN          0xc2 // VDV and VRH Command enable
#define TFT_LCD_320X480_CMD_VRHSET            0xc3 // VRH Set
#define TFT_LCD_320X480_CMD_VDVSET            0xc4 // VDV Set
#define TFT_LCD_320X480_CMD_VCMOFSET          0xc5 // VCOM Offset Set
#define TFT_LCD_320X480_CMD_FRCTR2            0xc6 // FR Control 2
#define TFT_LCD_320X480_CMD_CABCCTRL          0xc7 // CABC Control
#define TFT_LCD_320X480_CMD_REGSEL1           0xc8 // Register value selection 1
#define TFT_LCD_320X480_CMD_REGSEL2           0xca // Register value selection 2
#define TFT_LCD_320X480_CMD_PWMFRSEL          0xcc // PWM Frequency Selection
#define TFT_LCD_320X480_CMD_PWCTRL1           0xd0 // Power Control 1
#define TFT_LCD_320X480_CMD_VAPVANEN          0xd2 // Enable VAP/VAN signal output
#define TFT_LCD_320X480_CMD_CMD2EN            0xdf // Command 2 Enable
#define TFT_LCD_320X480_CMD_PVGAMCTRL         0xe0 // Positive Voltage Gamma Control
#define TFT_LCD_320X480_CMD_NVGAMCTRL         0xe1 // Negative voltage Gamma Control
#define TFT_LCD_320X480_CMD_DGMLUTR           0xe2 // Digital Gamma Look-up Table for Red
#define TFT_LCD_320X480_CMD_DGMLUTB           0xe3 // Digital Gamma Look-up Table for Blue
#define TFT_LCD_320X480_CMD_GATECTRL          0xe4 // Gate control
#define TFT_LCD_320X480_CMD_PWCTRL2           0xe8 // Power Control 2
#define TFT_LCD_320X480_CMD_EQCTRL            0xe9 // Equalize Time Control
#define TFT_LCD_320X480_CMD_PROMCTRL          0xec // Program Control
#define TFT_LCD_320X480_CMD_PROMEN            0xfa // Program Mode Enable
#define TFT_LCD_320X480_CMD_NVMSET            0xfc // NVM Setting
#define TFT_LCD_320X480_CMD_PROMACT           0xfe // Program Action

#define TFT_LCD_320X480_CMDLIST_END           0xff // End command (used for command list)


/**********************
 *      TYPEDEFS
 **********************/
struct TFT_LCD_320X480_driver;

typedef struct TFT_LCD_320X480_transaction_data {
	struct TFT_LCD_320X480_driver *driver;
	bool data;
} TFT_LCD_320X480_transaction_data_t;

typedef uint16_t TFT_LCD_320X480_color_t;

typedef struct TFT_LCD_320X480_driver {
	int pin_reset;
	int pin_dc;
	int pin_mosi;
	int pin_sclk;
	int spi_host;
	int dma_chan;
	uint8_t queue_fill;
	uint16_t display_width;
	uint16_t display_height;
	spi_device_handle_t spi;
	size_t buffer_size;
	TFT_LCD_320X480_transaction_data_t data;
	TFT_LCD_320X480_transaction_data_t command;
	TFT_LCD_320X480_color_t *buffer;
	TFT_LCD_320X480_color_t *buffer_primary;
	TFT_LCD_320X480_color_t *buffer_secondary;
	TFT_LCD_320X480_color_t *current_buffer;
	spi_transaction_t trans_a;
	spi_transaction_t trans_b;
} TFT_LCD_320X480_driver_t;

typedef struct TFT_LCD_320X480_command {
	uint8_t command;
	uint8_t wait_ms;
	uint8_t data_size;
	const uint8_t *data;
} TFT_LCD_320X480_command_t;


/**********************
 * GLOBAL PROTOTYPES
 **********************/

/*
 * Function:  TFT_LCD_320X480_init 
 * --------------------
 * 
 * Initialize the SPI peripheral and send the initialization sequence.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: True if the initialization suceed otherwise false.
 * 
 */
bool TFT_LCD_320X480_init(TFT_LCD_320X480_driver_t *driver);

/*
 * Function:  TFT_LCD_320X480_reset
 * --------------------
 * 
 * Reset the display
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void TFT_LCD_320X480_reset(TFT_LCD_320X480_driver_t *driver);

/*
 * Function:  TFT_LCD_320X480_fill_area
 * --------------------
 * 
 * Fill a area of the display with a selected color
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 	-color: 16 Bit hexadecimal color to fill the area.
 * 	-start_x: Start point on the X axis.
 * 	-start_y: Start point on the Y axis.
 * 	-width: Width of the area to be fill.
 * 	-height: Height of the area to be fill.
 * 
 * Returns: Nothing.
 * 
 */
void TFT_LCD_320X480_fill_area(TFT_LCD_320X480_driver_t *driver, TFT_LCD_320X480_color_t color, uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height);

/*
 * Function:  TFT_LCD_320X480_write_pixels 
 * --------------------
 * 
 * WIP
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void TFT_LCD_320X480_write_pixels(TFT_LCD_320X480_driver_t *driver, TFT_LCD_320X480_color_t *pixels, size_t length);

/*
 * Function:  TFT_LCD_320X480_write_lines 
 * --------------------
 * 
 * WIP
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void TFT_LCD_320X480_write_lines(TFT_LCD_320X480_driver_t *driver, int ypos, int xpos, int width, uint16_t *linedata, int lineCount);

/*
 * Function:  TFT_LCD_320X480_swap_buffers 
 * --------------------
 * 
 * The driver has two buffer, to allow send and render the image at the same type. This function
 * send the data of the actived buffer and change the pointer of current buffer to the next one.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void TFT_LCD_320X480_swap_buffers(TFT_LCD_320X480_driver_t *driver);

/*
 * Function:  TFT_LCD_320X480_set_window
 * --------------------
 * 
 * This screen allows partial update of the screen, so we can specified which part of the windows is going to change.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 	-start_x: X axis start point of the refresh zone.
 * 	-start_y: Y axis start point of the refresh zone. 
 *	-end_x: X axis end point of the refresh zone. 
 *	-end_y: Y axis end point of the refresh zone. 

 * Returns: Nothing.
 * 
 */
void TFT_LCD_320X480_set_window(TFT_LCD_320X480_driver_t *driver, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);

/*
 * Function:  TFT_LCD_320X480_set_endian 
 * --------------------
 * 
 * Depper explanation on the display_HAL.h file, but this function change the screen configuration from,
 * little endian message to big endian message.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */

void TFT_LCD_320X480_set_endian(TFT_LCD_320X480_driver_t *driver);

/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_100ASK_DRIVERS_H*/