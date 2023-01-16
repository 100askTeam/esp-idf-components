/**
 * @file epd_240x360_luts.h
 *
 */

#ifndef EPD_240X360_LUTS_H
#define EPD_240X360_LUTS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *     VARIABLES
 **********************/
extern const unsigned char epd_240x360_lut_R20_GC[56];
extern const unsigned char epd_240x360_lut_R21_GC[42];
extern const unsigned char epd_240x360_lut_R22_GC[56];
extern const unsigned char epd_240x360_lut_R23_GC[42];
extern const unsigned char epd_240x360_lut_R24_GC[42];

extern const unsigned char epd_240x360_lut_R20_DU[56];
extern const unsigned char epd_240x360_lut_R21_DU[42];
extern const unsigned char epd_240x360_lut_R22_DU[56];
extern const unsigned char epd_240x360_lut_R23_DU[42];
extern const unsigned char epd_240x360_lut_R24_DU[42];

extern const unsigned char epd_240x360_lut_vcom[42];
extern const unsigned char epd_240x360_lut_ww[42];
extern const unsigned char epd_240x360_lut_bw[42];
extern const unsigned char epd_240x360_lut_wb[42];
extern const unsigned char epd_240x360_lut_bb[42];



#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_240X360_LUTS_H*/