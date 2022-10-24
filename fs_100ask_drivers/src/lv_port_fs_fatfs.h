/**
 * @file lv_port_fs_fatfs.h
 *
 */

#ifndef LV_PORT_FS_FATFS_H
#define LV_PORT_FS_FATFS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if defined(CONFIG_LV_USE_100ASK_FS_FATFS)

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_port_fs_100ask_init(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_PORT_FS_FATFS_H*/

#endif /*Disable/Enable content*/
