/**
 * @file fs_100ask_hal.h
 *
 */

#ifndef EPD_100ASK_HAL_H
#define EPD_100ASK_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_FS_DRIVERS

/*********************
 *      DEFINES
 *********************/

#define EPD_100ASK_COLOR_WHITE     0xFF
#define EPD_100ASK_COLOR_BLACK     0x00


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void fs_100ask_hal_init(void);

void fs_100ask_hal_deinit(void);

sdmmc_card_t * fs_100ask_hal_get_mount_card(void);

/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_100ASK_HAL_H*/