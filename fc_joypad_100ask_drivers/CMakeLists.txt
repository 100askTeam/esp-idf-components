set(srcs "./src/fc_joypad_100ask_drivers.c")
set(includes "./src")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes})