/**
 * @file epd_100ask_drivers.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "epd_100ask_drivers.h"
#include "epd_100ask_luts.h"

#ifdef CONFIG_USE_100ASK_EPD

/*********************
 *      DEFINES
 *********************/
#define TAG "EPD_100ASK_DRIVERS"

#define EPD_100ASK_DISP_PIN_MOSI        CONFIG_EPD_100ASK_DISP_PIN_MOSI /*SDI*/
#define EPD_100ASK_DISP_PIN_MISO        CONFIG_EPD_100ASK_DISP_PIN_MISO 
#define EPD_100ASK_DISP_PIN_CLK         CONFIG_EPD_100ASK_DISP_PIN_CLK  /*SCLK*/
#define EPD_100ASK_DISP_PIN_CS          CONFIG_EPD_100ASK_DISP_PIN_CS   /*CS*/
#define EPD_100ASK_DISP_PIN_DC          CONFIG_EPD_100ASK_DISP_PIN_DC   /*D/C*/
#define EPD_100ASK_DISP_PIN_RST         CONFIG_EPD_100ASK_DISP_PIN_RST  /*RESET*/
#define EPD_100ASK_DISP_PIN_BUSY        CONFIG_EPD_100ASK_DISP_PIN_BUSY /*BUSY*/

#define EPD_100ASK_DISP_SPI_MODE        CONFIG_EPD_100ASK_DISP_SPI_MODE
#define EPD_100ASK_DISP_SPI_FREQUENCY   CONFIG_EPD_100ASK_DISP_SPI_FREQUENCY
#define EPD_100ASK_DISP_HEIGHT          CONFIG_EPD_100ASK_DISP_HEIGHT
#define EPD_100ASK_DISP_WIDTH           CONFIG_EPD_100ASK_DISP_WIDTH
#define EPD_100ASK_DISP_SPI_QUEUE_SIZE  7

/**********************
 *      TYPEDEFS
 **********************/
/*
 The EPD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} epd_init_cmd_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void epd_initialize(spi_device_handle_t spi);
static void epd_reset(spi_device_handle_t spi);
static void epd_send_data(spi_device_handle_t spi, const uint8_t *data, int len);
static void epd_send_cmd(spi_device_handle_t spi, const uint8_t cmd);
static void epd_spi_pre_transfer_callback(spi_transaction_t *t);

static void epd_100ask_lut5S(void);
static void epd_100ask_lutGC(void);
static void epd_100ask_lutDU(void);
static void epd_100ask_chkstatus(void);

/**********************
 *  STATIC VARIABLES
 **********************/
static int g_lut_flag;
static spi_device_handle_t g_spi;
static QueueHandle_t TransactionPool = NULL;


/**********************
 *   GLOBAL FUNCTIONS
 **********************/
//Initialize the display
void epd_100ask_init(void)
{
  esp_err_t ret;
  spi_bus_config_t buscfg={
      .miso_io_num=-1,
      .mosi_io_num=EPD_100ASK_DISP_PIN_MOSI,
      .sclk_io_num=EPD_100ASK_DISP_PIN_CLK,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1,
      .max_transfer_sz=(EPD_100ASK_DISP_WIDTH*EPD_100ASK_DISP_HEIGHT)/8
  };
  spi_device_interface_config_t devcfg={
      //.clock_speed_hz=EPD_100ASK_DISP_SPI_FREQUENCY,    //Clock out at 25 MHz
      .clock_speed_hz=25000000,    //Clock out at 25 MHz
      .mode=EPD_100ASK_DISP_SPI_MODE,                   //SPI mode 0
      .spics_io_num=EPD_100ASK_DISP_PIN_CS,             //CS pin
      .queue_size=EPD_100ASK_DISP_SPI_QUEUE_SIZE,       //We want to be able to queue 7 transactions at a time
      .pre_cb=epd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
  };
  //Initialize the SPI bus
  ret=spi_bus_initialize(SPI2_HOST, &buscfg, (spi_dma_chan_t)SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  //Attach the LCD to the SPI bus
  ret=spi_bus_add_device(SPI2_HOST, &devcfg, &g_spi);
  ESP_ERROR_CHECK(ret);
  
  /* create the transaction pool and fill it with ptrs to spi_transaction_ext_t to reuse */
	if(TransactionPool == NULL) {
		TransactionPool = xQueueCreate(EPD_100ASK_DISP_SPI_QUEUE_SIZE, sizeof(spi_transaction_ext_t*));
		assert(TransactionPool != NULL);
		for (size_t i = 0; i < EPD_100ASK_DISP_SPI_QUEUE_SIZE; i++)
		{
			spi_transaction_ext_t* pTransaction = (spi_transaction_ext_t*)heap_caps_malloc(sizeof(spi_transaction_ext_t), MALLOC_CAP_DMA);
			assert(pTransaction != NULL);
			memset(pTransaction, 0, sizeof(spi_transaction_ext_t));
			xQueueSend(TransactionPool, &pTransaction, portMAX_DELAY);
		}
	}

  //Initialize non-SPI GPIOs
  gpio_reset_pin(EPD_100ASK_DISP_PIN_BUSY);
  gpio_reset_pin(EPD_100ASK_DISP_PIN_RST);
  gpio_reset_pin(EPD_100ASK_DISP_PIN_DC);
  gpio_set_direction(EPD_100ASK_DISP_PIN_BUSY, GPIO_MODE_INPUT);
  gpio_set_direction(EPD_100ASK_DISP_PIN_RST, GPIO_MODE_OUTPUT);
  gpio_set_direction(EPD_100ASK_DISP_PIN_DC, GPIO_MODE_OUTPUT);
  
  //Initialize the EPD
  epd_initialize(g_spi);

  ESP_LOGI(TAG, "Initialized successfully!");
}



void epd_100ask_display_clear(uint8_t color)
{
  uint8_t * tmp_data = (uint8_t *)malloc(((EPD_100ASK_DISP_HEIGHT) * (EPD_100ASK_DISP_WIDTH) / 8) * sizeof(uint8_t));

  memset(tmp_data, color, ((EPD_100ASK_DISP_HEIGHT) * (EPD_100ASK_DISP_WIDTH) / 8));

  epd_send_cmd(g_spi, 0x10);
  epd_send_data(g_spi, tmp_data, (((EPD_100ASK_DISP_HEIGHT) * (EPD_100ASK_DISP_WIDTH) )/ 8));

  epd_send_cmd(g_spi, 0x13);
  epd_send_data(g_spi, tmp_data, (((EPD_100ASK_DISP_HEIGHT) * (EPD_100ASK_DISP_WIDTH)) / 8));

  free(tmp_data);
}


void epd_100ask_display_image(const uint8_t * picData, uint16_t w, uint16_t h)
{
    if(w > EPD_100ASK_DISP_HEIGHT)    w = EPD_100ASK_DISP_HEIGHT;
    if(h > EPD_100ASK_DISP_HEIGHT)    h = EPD_100ASK_DISP_HEIGHT;

    epd_send_cmd(g_spi, 0x13);
    epd_send_data(g_spi, picData, ((w * h) / 8));
}


// partial display 
void epd_100ask_display_partial(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, const uint8_t * data)
{
    uint16_t x_start1, x_end1, y_start1, y_start2, y_end1, y_end2;
    uint8_t tmp_data[8];

    x_start1 = x0;//转换为字节
    x_end1 = x0 + w - 1; 

    y_start1 = 0;
    y_start2 = y0;
    if(y0 >= 256)
    {
        y_start1 = y_start2 / 256;
        y_start2 = y_start2 % 256;
    }

    y_end1 = 0;
    y_end2 = y0 + h - 1;
    if(y_end2 >= 256)
    {
        y_end1 = y_end2 / 256;
        y_end2 = y_end2 % 256;		
    }

    /*********************************************************/	
    //需要重新复位和初始化设置!!!!
    /*********************************************************/	
    epd_reset(g_spi);

    tmp_data[0] = 0xF7;                 // Border
    epd_send_cmd(g_spi, 0x50);
    epd_send_data(g_spi, tmp_data, 1);

    /*********************************************************/		
    tmp_data[0] = 0xFF;                 // RES1 RES0 REG KW/R     UD    SHL   SHD_N  RST_N	
    tmp_data[1] = 0x01;                 // x x x VCMZ TS_AUTO TIGE NORG VC_LUTZ	
    epd_send_cmd(g_spi, 0x00);          // panel setting   PSR
    epd_send_data(g_spi, tmp_data, 2);

    tmp_data[0] = x_start1;             //x-start
    tmp_data[1] = x_end1;               //x-end
    tmp_data[2] = y_start1;
    tmp_data[3] = y_start2;             //y-start 
    tmp_data[4] = y_end1;
    tmp_data[5] = y_end2;               //y-end
    tmp_data[6] = 0x01;
    epd_send_cmd(g_spi, 0x91);          //This command makes the display enter partial mode
    epd_send_cmd(g_spi, 0x90);          //resolution setting
    epd_send_data(g_spi, tmp_data, 7);  

    epd_send_cmd(g_spi, 0x13);          // writes New data to SRAM.
    epd_send_data(g_spi, data, ((w * h) / 8));      // show data(image)

    epd_100ask_refresh(EPD_100ASK_LUT_DU);
    vTaskDelay(500 / portTICK_RATE_MS);

    /*********************************************************/	
    //需要重新复位和初始化设置!!!!
    /*********************************************************/	
    epd_reset(g_spi);

    tmp_data[0] = 0xD7;                 // Border
    epd_send_cmd(g_spi, 0x50);
    epd_send_data(g_spi, tmp_data, 1);
}

void epd_100ask_refresh(EPD_LUT_TYPE lut)
{
    uint8_t tmp_data[1];
    switch (lut)
    {
    case EPD_100ASK_LUT_GC:
        epd_100ask_lutGC();

        //DISPLAY REFRESH
        tmp_data[0] = 0xA5;
        epd_send_cmd(g_spi, 0x17);
        epd_send_data(g_spi, tmp_data, 1);

        epd_100ask_chkstatus();
        vTaskDelay(10 / portTICK_RATE_MS);
        break;

    case EPD_100ASK_LUT_DU:
        epd_100ask_lutDU();

        //DISPLAY REFRESH
        tmp_data[0] = 0xA5;
        epd_send_cmd(g_spi, 0x17);
        epd_send_data(g_spi, tmp_data, 1);
        
        epd_100ask_chkstatus();
        vTaskDelay(10 / portTICK_RATE_MS);

        epd_reset(g_spi);	

        tmp_data[0] = 0xD7;  // Border
        epd_send_cmd(g_spi, 0x50);
        epd_send_data(g_spi, tmp_data, 1);

        break;

    case EPD_100ASK_LUT_5S:
        epd_100ask_lut5S();
        break;

    default:
        return;
        break;
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

//Reset the display
static void epd_reset(spi_device_handle_t spi)
{
    epd_initialize(spi);
}



//Initialize the display
static void epd_initialize(spi_device_handle_t spi)
{
    g_lut_flag = 0;
  
    int cmd=0;
    const epd_init_cmd_t epd_init_cmds[]={
        /* panel setting   PSR 
        * RES1 RES0 REG KW/R     UD    SHL   SHD_N  RST_N
        * x x x VCMZ TS_AUTO TIGE NORG VC_LUTZ
        */
        {0x00, {0xFF, 0x01}, 2},
        /* POWER SETTING   PWR
        * cp1 keeps 1 frame, 1st frame enable
        * x x x VCOM_SLWE VGH[3:0]   VGH=20V, VGL=-20V
        * x x VSH[5:0]  VSH = 15V
        * x x VSL[5:0]  VSL=-15V
        * OPTEN VDHR[6:0]  VHDR=6.4V
        */
        {0x01, {0x03, 0x10, 0x3f, 0x3f, 0x03}, 5},
        /* T_VDS_OFF[1:0] 00=1 frame; 01=2 frame; 10=3 frame; 11=4 frame
        * booster soft start   BTST
        * BT_PHA[7:0]
        * BT_PHB[7:0]
        * x x BT_PHC[5:0]
        */
        {0x06, {0x37, 0x3D, 0x3D}, 3},
        /* */
        {0x60, {0x22}, 1},
        /* */
        {0x82, {0x07}, 1},
        /* */
        {0x30, {0x09}, 1},
        /* */
        {0xe3, {0x88}, 1},
        /* resoultion setting
        * HRES[7:3] 0 0 0
        * x x x x x x x VRES[8]
        * VRES[7:0]
        */
        {0x61, {0xf0, 0x01, 0x68}, 3},
        /*  */
        {0x50, {0xB7}, 1},
        {0, {0}, 0xff},
    };


    //Reset the display
    gpio_set_level(EPD_100ASK_DISP_PIN_RST, 1);
    vTaskDelay(20 / portTICK_RATE_MS);
    gpio_set_level(EPD_100ASK_DISP_PIN_RST, 0);
    vTaskDelay(20 / portTICK_RATE_MS);
    gpio_set_level(EPD_100ASK_DISP_PIN_RST, 1);
    vTaskDelay(20 / portTICK_RATE_MS);

    //Send all the commands
    while (epd_init_cmds[cmd].databytes != 0xff) {
        epd_send_cmd(spi, epd_init_cmds[cmd].cmd);
        epd_send_data(spi, epd_init_cmds[cmd].data, epd_init_cmds[cmd].databytes & 0x1F);
        if (epd_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
}



/* Send a command to the EPD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
static void epd_send_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    spi_transaction_t *presult;

	  while(uxQueueMessagesWaiting(TransactionPool) < EPD_100ASK_DISP_SPI_QUEUE_SIZE) {	/* service until the transaction reuse pool is full again */
        if (spi_device_get_trans_result(spi, &presult, 1) == ESP_OK) {
			xQueueSend(TransactionPool, &presult, portMAX_DELAY);
        }
    }

    //gpio_set_level(EPD_100ASK_DISP_PIN_DC, 0);	 /*Command mode*/

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.tx_buffer = &cmd;               //The data is the cmd itself
    t.user = (void*)0;                //D/C needs to be set to 0
    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

/* Send data to the EPD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
static void epd_send_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    spi_transaction_t *presult;

    if (len == 0) return;             //no need to send anything

	  while(uxQueueMessagesWaiting(TransactionPool) < EPD_100ASK_DISP_SPI_QUEUE_SIZE) {	/* service until the transaction reuse pool is full again */
        if (spi_device_get_trans_result(spi, &presult, 1) == ESP_OK) {
			xQueueSend(TransactionPool, &presult, portMAX_DELAY);
        }
    }


    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;               //Data
    t.user = (void*)1;                //D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}


//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
static void epd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(EPD_100ASK_DISP_PIN_DC, dc);
}



static void epd_100ask_chkstatus(void)
{
  while (0 == gpio_get_level(EPD_100ASK_DISP_PIN_BUSY))
    vTaskDelay(10 / portTICK_RATE_MS);
}


/*****************************LUT download*************************/
static void epd_100ask_lut5S(void)
{				
  //vcom
  epd_send_cmd(g_spi, 0x20);
  epd_send_data(g_spi, lut_vcom, 42);
						
  //red not use
  epd_send_cmd(g_spi, 0x21);
  epd_send_data(g_spi, lut_ww, 42);
				
  //wb w
  epd_send_cmd(g_spi, 0x24);
  epd_send_data(g_spi, lut_bb, 42);

  if (g_lut_flag == 0)
  {						
    //bb b
    epd_send_cmd(g_spi, 0x22);
    epd_send_data(g_spi, lut_bw, 42);
					
    //bw r
    epd_send_cmd(g_spi, 0x23);
    epd_send_data(g_spi, lut_wb, 42);


    g_lut_flag = 1;
  }
  else
  {
    //bb b
    epd_send_cmd(g_spi, 0x23);
    epd_send_data(g_spi, lut_bw, 42);
					
    //bw r
    epd_send_cmd(g_spi, 0x22);
    epd_send_data(g_spi, lut_wb, 42);

    g_lut_flag = 0;
  }
}


//LUT download
static void epd_100ask_lutGC(void)
{
  epd_send_cmd(g_spi, 0x20);
  epd_send_data(g_spi, lut_R20_GC, 56);

  //red not use
  epd_send_cmd(g_spi, 0x21);
  epd_send_data(g_spi, lut_R21_GC, 42);				

  //bb b
  epd_send_cmd(g_spi, 0x24);
  epd_send_data(g_spi, lut_R24_GC, 42);	

  if (g_lut_flag == 0)	
  {						
    //bw r
    epd_send_cmd(g_spi, 0x22);
    epd_send_data(g_spi, lut_R22_GC, 56);	
					
    //wb w
    epd_send_cmd(g_spi, 0x23);
    epd_send_data(g_spi, lut_R23_GC, 42);
    g_lut_flag = 1;
  }
  else
  {						
    //bw r
    epd_send_cmd(g_spi, 0x22);
    epd_send_data(g_spi, lut_R23_GC, 42);	

    //wb w
    epd_send_cmd(g_spi, 0x23);
    epd_send_data(g_spi, lut_R22_GC, 56);
    g_lut_flag = 0;

  }

}



//LUT download
static void epd_100ask_lutDU(void)
{		
    //vcom
    epd_send_cmd(g_spi, 0x20);
    epd_send_data(g_spi, lut_R20_DU, 56);	
		
    //red not use
    epd_send_cmd(g_spi, 0x21);
    epd_send_data(g_spi, lut_R21_DU, 42);	
						
    //bb b
    epd_send_cmd(g_spi, 0x24);
    epd_send_data(g_spi, lut_R24_DU, 42);	

    if (g_lut_flag == 0)	
    {
      //bw r
      epd_send_cmd(g_spi, 0x22);
      epd_send_data(g_spi, lut_R22_DU, 56);							

      //wb w
      epd_send_cmd(g_spi, 0x23);
      epd_send_data(g_spi, lut_R23_DU, 42);
      g_lut_flag = 1;
    }
    else
    {
      //bw r
      epd_send_cmd(g_spi, 0x22);
      epd_send_data(g_spi, lut_R23_DU, 42);							

      //wb w
      epd_send_cmd(g_spi, 0x23);
      epd_send_data(g_spi, lut_R22_DU, 56);
      g_lut_flag = 0;
    }
}

#endif
