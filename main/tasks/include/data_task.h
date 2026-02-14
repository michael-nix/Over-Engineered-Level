#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* `data_task` - task that receives IMU data, filters it, and sends the results
   to other tasks, such as the LED task.

   This task must be created after the IMU driver has been initialized and is
   capturing data; after the IMU task has been created; and, after the LED
   driver has been initialized.

   #### Parameters:
    - `pvParameters` - a pointer to an `IntervalBuffer` that holds IMU data.

   NOTE: the input parameter `IntervalBuffer` is shared with the IMU task, so
   locks or notifications must be used to ensure mutual exclusion.
*/
void data_task(void* pvParameters);

/* `data_task_notify` - send a notification to the `data_task`, primarily when
   new IMU data is available in the shared `IntervalBuffer`.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the `data_task` handle has not been set,
    - `ESP_OK` if the notification was successfully sent.
*/
esp_err_t data_task_notify();
