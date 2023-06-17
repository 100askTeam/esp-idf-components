set(srcs "./src/display_100ask_hal.c"
         "./src/drivers/lcd_100ask_spi.c"
         "./src/drivers/lcd_100ask_esp_lcd_panel.c"
         "./src/drivers/lcd_100ask_backlight.c"
         )
set(includes "./src" "./src/drivers")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES esp_lcd)
