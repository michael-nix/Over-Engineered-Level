idf_component_register(
    SRCS 
        "main.c"
        "tasks/imu_task.c"
        "tasks/data_task.c"
        "tasks/led_task.c"
    
    PRIV_REQUIRES 
        imu_driver
        filters
        led_driver
    
    INCLUDE_DIRS 
        ""
        "tasks/include"
)
