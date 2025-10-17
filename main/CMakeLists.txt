idf_component_register(
    SRCS "RideLink_main.cpp"
    REQUIRES driver
    PRIV_REQUIRES spi_flash
)
