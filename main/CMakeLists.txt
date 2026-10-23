idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp-zigbee-lib nvs_flash esp_timer lvgl
)
