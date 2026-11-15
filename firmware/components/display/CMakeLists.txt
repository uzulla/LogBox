idf_component_register(SRCS "display.cpp"
                       REQUIRES logger_status
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_lcd esp_timer)
