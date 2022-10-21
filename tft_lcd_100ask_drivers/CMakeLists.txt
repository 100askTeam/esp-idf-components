set(srcs "./src/st7796s.c"
         "./src/tft_lcd_100ask_drivers.c"
         "./src/tft_lcd_100ask_backlight.c"
         )
set(includes "./src")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes})