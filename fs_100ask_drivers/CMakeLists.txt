set(srcs "./src/fs_100ask_drivers.c" "./src/sd_spi_100ask.c" "./src/lv_port_fs_fatfs.c")
set(includes "./src")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes})