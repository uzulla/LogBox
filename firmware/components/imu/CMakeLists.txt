idf_component_register(SRCS "imu.cpp" "bmi270_config.cpp"
                       INCLUDE_DIRS include
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_rom)
