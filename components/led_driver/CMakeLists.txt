idf_component_register(
    SRCS 
    "led_driver.c"
    
    PRIV_REQUIRES
        esp_driver_rmt
    
    INCLUDE_DIRS 
    "include"
)
