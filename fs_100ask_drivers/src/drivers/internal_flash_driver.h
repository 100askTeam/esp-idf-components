/**
 * @file internal_flash_100ask_driver.h
 *
 */

#ifndef INTERNAL_FLASH_DRIVER_H
#define INTERNAL_FLASH_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_INTERNAL_FLASH

/*********************
 *      DEFINES
 *********************/


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
esp_err_t internal_flash_100ask_init(void);

/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*INTERNAL_FLASH_DRIVER_H*/