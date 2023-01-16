set(srcs "./src/tft_lcd_100ask_hal.c"
         "./src/drivers/tft_lcd_320x480.c"
         "./src/drivers/tft_lcd_100ask_backlight.c"
         )
set(includes "./src" "./src/drivers")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes})