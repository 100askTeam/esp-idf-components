/**
 * @file epd_100ask_paint.h
 *
 */
#ifndef EPD_100ASK_PAINT_H
#define EPD_100ASK_PAINT_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef CONFIG_USE_100ASK_EPD_PAINT

/*********************
 *      DEFINES
 *********************/
#define EPD_COLOR_WHITE     0xFF
#define EPD_COLOR_BLACK     0x00

/**
 * Display rotate
**/
#define ROTATE_0            0
#define ROTATE_90           90
#define ROTATE_180          180
#define ROTATE_270          270


/**********************
 *      TYPEDEFS
 **********************/
/**
 * Display Flip
**/
typedef enum {
    MIRROR_NONE  = 0x00,
    MIRROR_HORIZONTAL = 0x01,
    MIRROR_VERTICAL = 0x02,
    MIRROR_ORIGIN = 0x03,
} EPD_MIRROR_IMAGE;

#define EPD_MIRROR_IMAGE EPD_MIRROR_NONE

/**
 * The size of the point
**/
typedef enum {
    DOT_PIXEL_1X1  = 1,		// 1 x 1
    DOT_PIXEL_2X2  , 		// 2 X 2
    DOT_PIXEL_3X3  ,		// 3 X 3
    DOT_PIXEL_4X4  ,		// 4 X 4
    DOT_PIXEL_5X5  , 		// 5 X 5
    DOT_PIXEL_6X6  , 		// 6 X 6
    DOT_PIXEL_7X7  , 		// 7 X 7
    DOT_PIXEL_8X8  , 		// 8 X 8
} DOT_PIXEL;
#define DOT_PIXEL_DFT  DOT_PIXEL_1X1  //Default dot pilex

/**
 * Point size fill style
**/
typedef enum {
    DOT_FILL_AROUND  = 1,		// dot pixel 1 x 1
    DOT_FILL_RIGHTUP  , 		// dot pixel 2 X 2
} DOT_STYLE;
#define DOT_STYLE_DFT  DOT_FILL_AROUND  //Default dot pilex

/**
 * Line style, solid or dashed
**/
typedef enum {
    LINE_STYLE_SOLID = 0,
    LINE_STYLE_DOTTED,
} LINE_STYLE;

/**
 * Whether the graphic is filled
**/
typedef enum {
    DRAW_FILL_EMPTY = 0,
    DRAW_FILL_FULL,
} DRAW_FILL;


/**********************
 * GLOBAL PROTOTYPES
 **********************/
void epd_100ask_paint_init(unsigned char * image, unsigned char color);

void epd_100ask_paint_clear(uint8_t color);

void epd_100ask_paint_set_pixel(uint16_t Xpoint, uint16_t Ypoint, uint8_t Color);

void epd_100ask_paint_draw_point(uint16_t Xpoint, uint16_t Ypoint, uint8_t Color,
                     DOT_PIXEL Dot_Pixel, DOT_STYLE DOT_STYLE);

void epd_100ask_paint_draw_line(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,
                    uint8_t Color, DOT_PIXEL Dot_Pixel, LINE_STYLE Line_Style);

void epd_100ask_paint_draw_rectangle(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,
                         uint8_t Color, DOT_PIXEL Dot_Pixel, DRAW_FILL Filled);

void epd_100ask_paint_draw_circle(uint16_t X_Center, uint16_t Y_Center, uint16_t Radius,
                      uint8_t Color, DOT_PIXEL Dot_Pixel, DRAW_FILL  Draw_Fill);


/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*EPD_100ASK_PAINT_H*/