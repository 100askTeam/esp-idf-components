file(GLOB_RECURSE SOURCES   epd_100ask_drivers/src/*.c
                            epd_100ask_paint/src/*.c
                            tft_lcd_100ask_drivers/src/*.c
                            fc_joypad_100ask_drivers/src/*.c
                            lvgl/demos/*.c
                            lvgl/examples/*.c
                            lvgl/src/*.c
                            )


idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS . 
                            epd_100ask_drivers/src
                            epd_100ask_paint/src
                            tft_lcd_100ask_drivers/src/
                            fc_joypad_100ask_drivers/src
                            lvgl/demos
                            lvgl/examples
                            lvgl/src
                            )

