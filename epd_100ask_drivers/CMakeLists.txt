set(srcs "./src/epd_100ask_hal.c" "./src/luts/epd_240x360_luts.c" "./src/drivers/epd_240x360_spi_driver.c")
set(includes "./src" "./src/drivers" "./src/luts")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes})