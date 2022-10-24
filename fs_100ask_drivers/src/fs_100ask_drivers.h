/**
 * @file epd_100ask_drivers.h
 *
 */

#ifndef EPD_100ASK_DRIVERS_H
#define EPD_100ASK_DRIVERS_H

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

/**
 * Refresh type
**/
typedef enum {
    EPD_100ASK_LUT_GC,         /* 全刷 */
    EPD_100ASK_LUT_DU,         /* 局刷 */
    EPD_100ASK_LUT_5S,         /* ... */
    _EPD_100ASK_LUT_LAST,      /** Number of default events*/
} EPD_LUT_TYPE;


/**********************
 * GLOBAL PROTOTYPES
 **********************/
void fs_100ask_drivers_init(void);

void fs_100ask_drivers_release(void);

/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_100ASK_DRIVERS_H*/