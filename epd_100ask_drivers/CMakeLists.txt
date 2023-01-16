set(srcs "./src/epd_100ask_hal.c" "./src/epd_luts/epd_240x360_luts.c" "./src/epd_drivers/epd_240x360_spi_driver.c")
set(includes "./src" "./src/epd_drivers" "./src/epd_luts")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes})