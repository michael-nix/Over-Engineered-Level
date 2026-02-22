#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "data_task.h"
#include "filters.h"
#include "imu_driver.h"
#include "imu_task.h"
#include "led_driver.h"
#include "led_task.h"

#include <math.h>

#define TASK_HIGHEST_PRIORITY  3
#define TASK_STANDARD_PRIORITY 2

static IntervalBuffer imu_data;

void app_main(void)
{
    ESP_ERROR_CHECK(imu_initialize(imu_interrupt_handler, NULL));
    ESP_ERROR_CHECK(led_initialize());

    imu_data = initialize_interval_buffer(
        (size_t)ceilf(FILTER_GROUP_DELAY_SAMPLES / IMU_SAMPLES_PER_PERIOD) + 1,
        IMU_SAMPLES_PER_PERIOD, sizeof(IMUData));

    TaskHandle_t imu_task_handle = 0;
    BaseType_t error = xTaskCreate(imu_task, "imu_task", 2048, &imu_data,
        TASK_HIGHEST_PRIORITY, &imu_task_handle);
    if (pdPASS != error)
    {
        ESP_LOGE(__func__,
            "Could not create and launch the IMU task! Hanging main task...");
        vTaskDelay(portMAX_DELAY);
    }

    TaskHandle_t data_task_handle = 0;
    error = xTaskCreate(data_task, "data_task", 3072, &imu_data,
        TASK_HIGHEST_PRIORITY, &data_task_handle);
    if (pdPASS != error)
    {
        ESP_LOGE(__func__,
            "Could not create and launch the Data task! Hanging main task...");
        vTaskDelay(portMAX_DELAY);
    }

    TaskHandle_t led_task_handle = 0;
    error = xTaskCreate(led_task, "led_task", 1024, NULL,
        TASK_STANDARD_PRIORITY, &led_task_handle);
    if (pdPASS != error)
    {
        ESP_LOGE(__func__,
            "Could not create and launch the LED task! Hanging main task...");
        vTaskDelay(portMAX_DELAY);
    }

    while (true)
    {
        ESP_LOGI(__func__, "IMU task high watermark: %d words",
            uxTaskGetStackHighWaterMark2(imu_task_handle));

        ESP_LOGI(__func__, "Data task high watermark: %d words",
            uxTaskGetStackHighWaterMark2(data_task_handle));

        ESP_LOGI(__func__, "LED task high watermark: %d words",
            uxTaskGetStackHighWaterMark2(led_task_handle));

        ESP_LOGI(__func__, "Minimum free heap size: %d bytes\n",
            heap_caps_get_minimum_free_size(0));

        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
