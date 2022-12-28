set(srcs "./src/fs_100ask_hal.c" "./src/internal_flash_100ask_drivers.c" "./src/lv_port_fs_fatfs.c" "./src/sd_spi_100ask_drivers.c")
set(includes "./src")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes})
