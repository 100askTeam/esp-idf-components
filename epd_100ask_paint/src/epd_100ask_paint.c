/**
 * @file epd_100ask_paint.c
 *
 */

/*********************
 *      INCLUDES
 *********************/



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "epd_100ask_paint.h"

#ifdef CONFIG_USE_100ASK_EPD_PAINT

/*********************
 *      DEFINES
 *********************/
#define TAG "EPD_100ASK_PAINT"

#define EPD_100ASK_PAINT_HEIGHT          CONFIG_EPD_100ASK_PAINT_HEIGHT
#define EPD_100ASK_PAINT_WIDTH           CONFIG_EPD_100ASK_PAINT_WIDTH

/**********************
 *      TYPEDEFS
 **********************/
typedef struct {
    unsigned char * image;
    unsigned char color;
    unsigned int width;
    unsigned int height;
    unsigned int widthMemory;
    unsigned int heightMemory;
    unsigned int rotate;
    unsigned int mirror;
    unsigned int widthByte;
    unsigned int heightByte;
} type_epd_paint_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static type_epd_paint_t g_paint;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void epd_100ask_paint_init(unsigned char * image, unsigned char color)
{
  uint16_t w = EPD_100ASK_PAINT_WIDTH;
  uint16_t h = EPD_100ASK_PAINT_HEIGHT;

  g_paint.image = image;
  g_paint.color = color;
  g_paint.rotate = 0;

  g_paint.widthMemory = w;
  g_paint.heightMemory = h;
  
  g_paint.widthByte = (w % 8 == 0)? (w / 8 ): (w / 8 + 1);
  g_paint.heightByte = h;

  g_paint.mirror = MIRROR_NONE;

  if(g_paint.rotate == ROTATE_0 || g_paint.rotate == ROTATE_180) {
      g_paint.width = w;
      g_paint.height = h;
  } else {
      g_paint.width = h;
      g_paint.height = w;
  }

  ESP_LOGI(TAG, "Initialized successfully!");
}

/******************************************************************************
function:	Clear the color of the picture
parameter:
    Color   :   Painted colors
******************************************************************************/
void epd_100ask_paint_clear(uint8_t color)
{
	uint16_t x, y;
	uint32_t addr;
    for (y = 0; y < g_paint.heightByte; y++) {
        for (x = 0; x < g_paint.widthByte; x++ ) {//8 pixel =  1 byte
            addr = x + y * (g_paint.widthByte);
            g_paint.image[addr] = color;
        }
    }
}

/******************************************************************************
function:	Draw Pixels
parameter:
    Xpoint  :   At point X
    Ypoint  :   At point Y
    Color   :   Painted colors
******************************************************************************/
void epd_100ask_paint_set_pixel(uint16_t Xpoint, uint16_t Ypoint, uint8_t Color)
{
	uint16_t X, Y;
	uint32_t Addr;
	uint8_t Rdata;

    if(Xpoint > g_paint.width || Ypoint > g_paint.height){
        //Debug("Exceeding display boundaries\r\n");
        return;
    }      
    
    switch(g_paint.rotate) {
    case 0:
        X = Xpoint;
        Y = Ypoint;  
        break;
    case 90:
        X = g_paint.widthMemory - Ypoint - 1;
        Y = Xpoint;
        break;
    case 180:
        X = g_paint.widthMemory - Xpoint - 1;
        Y = g_paint.heightMemory - Ypoint - 1;
        break;
    case 270:
        X = Ypoint;
        Y = g_paint.heightMemory - Xpoint - 1;
        break;
		
    default:
        return;
    }
    
    switch(g_paint.mirror) {
    case MIRROR_NONE:
        break;
    case MIRROR_HORIZONTAL:
        X = g_paint.widthMemory - X - 1;
        break;
    case MIRROR_VERTICAL:
        Y = g_paint.heightMemory - Y - 1;
        break;
    case MIRROR_ORIGIN:
        X = g_paint.widthMemory - X - 1;
        Y = g_paint.heightMemory - Y - 1;
        break;
    default:
        return;
    }

    if(X > g_paint.widthMemory || Y > g_paint.heightMemory){
        //Debug("Exceeding display boundaries\r\n");
        return;
    }
    
    Addr = X / 8 + Y * g_paint.widthByte;
    Rdata = g_paint.image[Addr];
    if(Color == EPD_COLOR_BLACK)
        g_paint.image[Addr] = Rdata & ~(0x80 >> (X % 8));
    else
        g_paint.image[Addr] = Rdata | (0x80 >> (X % 8));
}

/******************************************************************************
function:	Draw Point(Xpoint, Ypoint) Fill the color
parameter:
    Xpoint		:   The Xpoint coordinate of the point
    Ypoint		:   The Ypoint coordinate of the point
    Color		:   Set color
    Dot_Pixel	:	point size
******************************************************************************/
void epd_100ask_paint_draw_point(uint16_t Xpoint, uint16_t Ypoint, uint8_t Color,
                     DOT_PIXEL Dot_Pixel, DOT_STYLE DOT_STYLE)
{
	int16_t XDir_Num , YDir_Num;
    if (Xpoint > g_paint.width || Ypoint > g_paint.height) {
        //Debug("Paint_DrawPoint Input exceeds the normal display range\r\n");
        return;
    }

    if (DOT_STYLE == DOT_FILL_AROUND) {
        for (XDir_Num = 0; XDir_Num < 2 * Dot_Pixel - 1; XDir_Num++) {
            for (YDir_Num = 0; YDir_Num < 2 * Dot_Pixel - 1; YDir_Num++) {
                if(Xpoint + XDir_Num - Dot_Pixel < 0 || Ypoint + YDir_Num - Dot_Pixel < 0)
                    break;
                epd_100ask_paint_set_pixel(Xpoint + XDir_Num - Dot_Pixel, Ypoint + YDir_Num - Dot_Pixel, Color);
            }
        }
    } else {
        for (XDir_Num = 0; XDir_Num <  Dot_Pixel; XDir_Num++) {
            for (YDir_Num = 0; YDir_Num <  Dot_Pixel; YDir_Num++) {
                epd_100ask_paint_set_pixel(Xpoint + XDir_Num - 1, Ypoint + YDir_Num - 1, Color);
            }
        }
    }
}


/******************************************************************************
function:	Draw a line of arbitrary slope
parameter:
    Xstart ：Starting Xpoint point coordinates
    Ystart ：Starting Xpoint point coordinates
    Xend   ：End point Xpoint coordinate
    Yend   ：End point Ypoint coordinate
    Color  ：The color of the line segment
******************************************************************************/
void epd_100ask_paint_draw_line(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,
                    uint8_t Color, DOT_PIXEL Dot_Pixel, LINE_STYLE Line_Style)
{    
    uint16_t Xpoint, Ypoint;
    int dx, dy;
    int XAddway,YAddway;
    int Esp;
    char Dotted_Len;

    if (Xstart > g_paint.width || Ystart > g_paint.height ||
        Xend > g_paint.width || Yend > g_paint.height) {
        //Debug("Paint_DrawLine Input exceeds the normal display range\r\n");
        return;
    }

    Xpoint = Xstart;
    Ypoint = Ystart;
    dx = (int)Xend - (int)Xstart >= 0 ? Xend - Xstart : Xstart - Xend;
    dy = (int)Yend - (int)Ystart <= 0 ? Yend - Ystart : Ystart - Yend;

    // Increment direction, 1 is positive, -1 is counter;
    XAddway = Xstart < Xend ? 1 : -1;
    YAddway = Ystart < Yend ? 1 : -1;

    //Cumulative error
    Esp = dx + dy;
    Dotted_Len = 0;

    for (;;) {
        Dotted_Len++;
        //Painted dotted line, 2 point is really virtual
        if (Line_Style == LINE_STYLE_DOTTED && Dotted_Len % 3 == 0) {
            //Debug("LINE_DOTTED\r\n");
            epd_100ask_paint_draw_point(Xpoint, Ypoint, EPD_COLOR_WHITE, Dot_Pixel, DOT_FILL_AROUND);
            Dotted_Len = 0;
        } else {
            epd_100ask_paint_draw_point(Xpoint, Ypoint, Color, Dot_Pixel, DOT_FILL_AROUND);
        }
        if (2 * Esp >= dy) {
            if (Xpoint == Xend)
                break;
            Esp += dy;
            Xpoint += XAddway;
        }
        if (2 * Esp <= dx) {
            if (Ypoint == Yend)
                break;
            Esp += dx;
            Ypoint += YAddway;
        }
    }
}


/******************************************************************************
function:	Draw a rectangle
parameter:
    Xstart ：Rectangular  Starting Xpoint point coordinates
    Ystart ：Rectangular  Starting Xpoint point coordinates
    Xend   ：Rectangular  End point Xpoint coordinate
    Yend   ：Rectangular  End point Ypoint coordinate
    Color  ：The color of the Rectangular segment
    Filled : Whether it is filled--- 1 solid 0：empty
******************************************************************************/
void epd_100ask_paint_draw_rectangle(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend,
                         uint8_t Color, DOT_PIXEL Dot_Pixel, DRAW_FILL Filled)
{
	uint16_t Ypoint;
    if (Xstart > g_paint.width || Ystart > g_paint.height ||
        Xend > g_paint.width || Yend > g_paint.height) {
        //Debug("Input exceeds the normal display range\r\n");
        return;
    }

    if (Filled ) {
        for(Ypoint = Ystart; Ypoint < Yend; Ypoint++) {
            epd_100ask_paint_draw_line(Xstart, Ypoint, Xend, Ypoint, Color , Dot_Pixel, LINE_STYLE_SOLID);
        }
    } else {
        epd_100ask_paint_draw_line(Xstart, Ystart, Xend, Ystart, Color, Dot_Pixel, LINE_STYLE_SOLID);
        epd_100ask_paint_draw_line(Xstart, Ystart, Xstart, Yend, Color, Dot_Pixel, LINE_STYLE_SOLID);
        epd_100ask_paint_draw_line(Xend, Yend, Xend, Ystart, Color, Dot_Pixel, LINE_STYLE_SOLID);
        epd_100ask_paint_draw_line(Xend, Yend, Xstart, Yend, Color, Dot_Pixel, LINE_STYLE_SOLID);
    }
}


/******************************************************************************
function:	Use the 8-point method to draw a circle of the
            specified size at the specified position->
parameter:
    X_Center  ：Center X coordinate
    Y_Center  ：Center Y coordinate
    Radius    ：circle Radius
    Color     ：The color of the ：circle segment
    Filled    : Whether it is filled: 1 filling 0：Do not
******************************************************************************/
void epd_100ask_paint_draw_circle(uint16_t X_Center, uint16_t Y_Center, uint16_t Radius,
                      uint8_t Color, DOT_PIXEL Dot_Pixel, DRAW_FILL  Draw_Fill)
{
	int16_t Esp, sCountY;
	int16_t XCurrent, YCurrent;
    if (X_Center > g_paint.width || Y_Center >= g_paint.height) {
        //Debug("Paint_DrawCircle Input exceeds the normal display range\r\n");
        return;
    }

    //Draw a circle from(0, R) as a starting point
    XCurrent = 0;
    YCurrent = Radius;

    //Cumulative error,judge the next point of the logo
    Esp = 3 - (Radius << 1 );
    if (Draw_Fill == DRAW_FILL_FULL) {
        while (XCurrent <= YCurrent ) { //Realistic circles
            for (sCountY = XCurrent; sCountY <= YCurrent; sCountY ++ ) {
                epd_100ask_paint_draw_point(X_Center + XCurrent, Y_Center + sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//1
                epd_100ask_paint_draw_point(X_Center - XCurrent, Y_Center + sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//2
                epd_100ask_paint_draw_point(X_Center - sCountY, Y_Center + XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//3
                epd_100ask_paint_draw_point(X_Center - sCountY, Y_Center - XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//4
                epd_100ask_paint_draw_point(X_Center - XCurrent, Y_Center - sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//5
                epd_100ask_paint_draw_point(X_Center + XCurrent, Y_Center - sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//6
                epd_100ask_paint_draw_point(X_Center + sCountY, Y_Center - XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//7
                epd_100ask_paint_draw_point(X_Center + sCountY, Y_Center + XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);
            }
            if (Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    } else { //Draw a hollow circle
        while (XCurrent <= YCurrent ) {
            epd_100ask_paint_draw_point(X_Center + XCurrent, Y_Center + YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//1
            epd_100ask_paint_draw_point(X_Center - XCurrent, Y_Center + YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//2
            epd_100ask_paint_draw_point(X_Center - YCurrent, Y_Center + XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//3
            epd_100ask_paint_draw_point(X_Center - YCurrent, Y_Center - XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//4
            epd_100ask_paint_draw_point(X_Center - XCurrent, Y_Center - YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//5
            epd_100ask_paint_draw_point(X_Center + XCurrent, Y_Center - YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//6
            epd_100ask_paint_draw_point(X_Center + YCurrent, Y_Center - XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//7
            epd_100ask_paint_draw_point(X_Center + YCurrent, Y_Center + XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT);//0

            if (Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    }
}


/**********************
 *   STATIC FUNCTIONS
 **********************/


#endif