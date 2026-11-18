idf_component_register(
    SRCS "gnss.cpp" "ubx.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_uart esp_timer esp_pm)
