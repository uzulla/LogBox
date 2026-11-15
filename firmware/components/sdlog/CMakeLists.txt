idf_component_register(SRCS "sdlog.cpp"
                       INCLUDE_DIRS include
                       REQUIRES record
                       PRIV_REQUIRES fatfs sdmmc esp_driver_spi esp_driver_sdspi nvs_flash esp_timer)
