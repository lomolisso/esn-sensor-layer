idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS ".")

idf_build_set_property(COMPILE_OPTIONS "-DMAX_MEM_LEVEL=7" APPEND)
