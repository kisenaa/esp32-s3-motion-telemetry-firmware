idf_component_register(
    SRCS
        "mma8451q.c"
    INCLUDE_DIRS
        "include"
    PRIV_REQUIRES
        esp_driver_i2c
)