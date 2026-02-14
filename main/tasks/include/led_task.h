#pragma once

typedef struct led_data
{
    float angle;
    float intensity;
} LEDData;

/* `led_task` - task that manages the WS-2812 LED strip, updating the LED
   colours based on data received from the data task.

   This task must be created after the LED driver has been initialized.

   #### Parameters:
    - `pvParameters` - unused.
*/
void led_task(void* pvParameters);

/* `send_data_to_led_task` - sends `LEDData` to the LED task.

   #### Parameters:
    - `data` - pointer to the `LEDData` to send to the LED task.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the LED task queue has not been created,
    - `ESP_OK` if the data was successfully sent to the LED task.
*/
esp_err_t send_data_to_led_task(LEDData* data);
