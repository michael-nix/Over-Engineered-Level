idf_component_register(
    SRCS    
        "imu_driver.c"

    PRIV_REQUIRES
        esp_driver_i2c

    REQUIRES
        esp_common
        esp_driver_gpio

    INCLUDE_DIRS 
        "include"
)
