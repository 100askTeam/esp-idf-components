file(GLOB_RECURSE SOURCES   epd_100ask_drivers/src/*.c
                            epd_100ask_paint/src/*.c
                            display_100ask_driver/src/*.c
                            fc_joypad_100ask_drivers/src/*.c
                            fs_100ask_drivers/src/*.c
                            lv_lib_100ask/src/*.c
                            lv_lib_100ask/test/*.c
                            lvgl/demos/*.c
                            lvgl/examples/*.c
                            lvgl/src/*.c
                            )


idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS . 
                            epd_100ask_drivers/src
                            epd_100ask_paint/src
                            display_100ask_driver/src
                            fc_joypad_100ask_drivers/src
                            fs_100ask_drivers/src
                            lv_lib_100ask
                            lv_lib_100ask/src
                            lv_lib_100ask/test
                            lvgl/demos
                            lvgl/examples
                            lvgl/src
                            REQUIRES fatfs)

