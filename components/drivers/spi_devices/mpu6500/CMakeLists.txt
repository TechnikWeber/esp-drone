idf_component_register(SRCS "mpu6500.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_spi
                       PRIV_REQUIRES spi_bus esp_driver_gpio)
