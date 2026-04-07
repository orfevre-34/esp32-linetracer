idf_component_register(SRCS "linetrace_car.cpp"
    INCLUDE_DIRS "."
    REQUIRES drivers
    PRIV_REQUIRES nvs_flash
)
