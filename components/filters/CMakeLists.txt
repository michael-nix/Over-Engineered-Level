idf_component_register(
    SRCS 
        "filters.c"
        "attitude.c"
                    
    INCLUDE_DIRS 
        "include"

    REQUIRES
        esp_common
)
