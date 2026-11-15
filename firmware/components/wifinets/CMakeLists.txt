idf_component_register(SRCS "wifinets.cpp" "wifinets_store.cpp"
                       INCLUDE_DIRS include
                       PRIV_REQUIRES nvs_flash freertos)
