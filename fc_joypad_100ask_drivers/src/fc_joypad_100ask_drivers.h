/**
 * @file fc_joypad_100ask_drivers.h
 *
 */
#ifndef FC_JOYPAD_100ASK_DRIVERS_H
#define FC_JOYPAD_100ASK_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "driver/gpio.h"

/*********************
 *      DEFINES
 *********************/


/**********************
 *      TYPEDEFS
 **********************/
typedef enum {
    FC_JOYPAD_100ASK_BTN_A = 0,
    FC_JOYPAD_100ASK_BTN_B,
    FC_JOYPAD_100ASK_BTN_SELECT,
    FC_JOYPAD_100ASK_BTN_START,
    FC_JOYPAD_100ASK_BTN_UP,
    FC_JOYPAD_100ASK_BTN_DOWN,
    FC_JOYPAD_100ASK_BTN_LEFT,
    FC_JOYPAD_100ASK_BTN_RIGHT,
} fc_joypad_btn_t;

typedef enum {
    FC_JOYPAD_100ASK_RELEASE = 0,
    FC_JOYPAD_100ASK_PRESSED,
} fc_joypad_state_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void fc_joypad_100ask_init(void);
void fc_joypad_100ask_read(void);
bool fc_joypad_100ask_is_pressed(fc_joypad_btn_t btn);
bool fc_joypad_100ask_is_release(fc_joypad_btn_t btn);
fc_joypad_state_t fc_joypad_100ask_state(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*FC_JOYPAD_100ASK_DRIVERS_H*/