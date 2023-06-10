set(srcs "./src/fs_100ask_hal.c" "./src/lv_port_fs_fatfs.c" "./src/drivers/internal_flash_driver.c" "./src/drivers/sd_spi_driver.c")
set(includes "./src" "./src/drivers")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES fatfs lvgl)
