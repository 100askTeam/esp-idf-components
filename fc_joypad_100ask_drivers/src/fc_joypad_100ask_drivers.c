/**
 * @file fc_joypad_100ask_drivers.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "fc_joypad_100ask_drivers.h"

#ifdef CONFIG_USE_100ASK_FC_JOYPAD

/*********************
 *      DEFINES
 *********************/
#define TAG "FC_JOYPAD_100ASK_DRIVERS"

#define FC_JOYPAD_CLOCK_PIN        CONFIG_100ASK_FC_JOYPAD_CLOCK_PIN    /*ID*/
#define FC_JOYPAD_LATCH_PIN        CONFIG_100ASK_FC_JOYPAD_LATCH_PIN    /*D+*/
#define FC_JOYPAD_DATA_PIN         CONFIG_100ASK_FC_JOYPAD_DATA_PIN     /*D-*/


/**********************
 *      TYPEDEFS
 **********************/
typedef struct {    
    char prev_player_value;
    char player;
    bool joypad_state;
} fc_joypad_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static fc_joypad_t g_fc_joypad;


/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void fc_joypad_100ask_init(void)
{

    ESP_LOGI(TAG, "Configured FC joyapd GPIO!");
    //printf("clock_pin:%d, latch_pin:%d, data_pin:%d\n", FC_JOYPAD_CLOCK_PIN, FC_JOYPAD_LATCH_PIN, FC_JOYPAD_DATA_PIN);
    
    gpio_reset_pin(FC_JOYPAD_CLOCK_PIN);
    gpio_reset_pin(FC_JOYPAD_LATCH_PIN);
    gpio_reset_pin(FC_JOYPAD_DATA_PIN);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(FC_JOYPAD_CLOCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(FC_JOYPAD_LATCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(FC_JOYPAD_DATA_PIN,  GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "FC joyapd initialized successfully!");
}

void fc_joypad_100ask_read(void)
{
    uint8_t count = 0;
    int8_t tmp_value = g_fc_joypad.player;

    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(FC_JOYPAD_LATCH_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5)); // ms
    gpio_set_level(FC_JOYPAD_LATCH_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(5)); // ms

    g_fc_joypad.joypad_state = false;
    g_fc_joypad.player = 0;
    for (count = 0; count < 8; count++)
    {
        if (gpio_get_level(FC_JOYPAD_DATA_PIN) == 0)
        {
            if (!(g_fc_joypad.joypad_state))
                g_fc_joypad.prev_player_value = tmp_value;

            g_fc_joypad.player |= (1 << count);
            g_fc_joypad.joypad_state = true;
        }  
        
        gpio_set_level(FC_JOYPAD_CLOCK_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(5)); // ms
        gpio_set_level(FC_JOYPAD_CLOCK_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(5)); // ms
    }
}


bool fc_joypad_100ask_is_pressed(fc_joypad_100ask_btn_t btn)
{
    if ((((uint8_t)1 << btn) & g_fc_joypad.player) >= 1)
        return true;
    else
        return false;
}


bool fc_joypad_100ask_is_release(fc_joypad_100ask_btn_t btn)
{
    if ((((uint8_t)1 << btn) & g_fc_joypad.prev_player_value) >= 1)
        return true;
    else
        return false;
}


fc_joypad_100ask_state_t fc_joypad_100ask_state(void)
{
    if(g_fc_joypad.joypad_state)
        return  FC_JOYPAD_100ASK_PRESSED;
    else
        return FC_JOYPAD_100ASK_RELEASE;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
