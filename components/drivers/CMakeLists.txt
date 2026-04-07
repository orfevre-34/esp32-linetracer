idf_component_register(
    SRCS "adc_reader.cpp" "button.cpp" "buzzer.cpp" "motor_driver.cpp" "pl9823.cpp" "ssd1306.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc esp_timer
)
