#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "led_driver.h"
#include "led_task.h"
#include "ws2812.h"

#include <math.h>

#define LED_INTENSITY       8
#define LED_RED_THRESHOLD   0.25f
#define LED_GREEN_THRESHOLD 0.05f

#define PI 3.141592653589793f

static QueueHandle_t led_task_queue = NULL;

void led_task(void* pvParameters)
{
    LEDPixel pixels[WS2818_NUM_PIXELS] = {0};

    LEDData data = {0};
    led_task_queue = xQueueCreate(2, sizeof(data));

    while (true)
    {
        xQueueReceive(led_task_queue, &data, portMAX_DELAY);
        float angle = data.angle;
        float intensity = data.intensity;

        if (LED_GREEN_THRESHOLD > intensity)
        {
            for (size_t pixel = 0; pixel < WS2818_NUM_PIXELS; pixel++)
            {
                pixels[pixel].g = LED_INTENSITY;
            }
        }
        else
        {
            LEDPixel colour = {0};
            if (LED_RED_THRESHOLD >= intensity)
            {
                colour.g = LED_INTENSITY;
                colour.r = LED_INTENSITY;
            }
            else
            {
                colour.r = LED_INTENSITY;
            }

            angle = (((angle / 2.0f / PI) + 0.5) * WS2818_NUM_PIXELS);

            uint8_t pixel = (uint8_t)(angle) % WS2818_NUM_PIXELS;
            pixels[pixel] = colour;

            pixel = (pixel + 1) % WS2818_NUM_PIXELS;
            pixels[pixel] = colour;

            pixel = (pixel + WS2818_NUM_PIXELS - 2) % WS2818_NUM_PIXELS;
            pixels[pixel] = colour;
        }

        led_set_pixels(pixels, WS2818_NUM_PIXELS);

        memset(pixels, 0, sizeof(LEDPixel) * WS2818_NUM_PIXELS);
    }
}

esp_err_t send_data_to_led_task(LEDData* data)
{
    if (NULL == led_task_queue)
    {
        ESP_LOGE(__func__,
            "Could not send angle to LED task. Queue is uninitialized!");

        return ESP_ERR_INVALID_ARG;
    }

    BaseType_t error = xQueueSend(led_task_queue, data, 0);
    if (pdTRUE != error)
    {
        ESP_LOGE(__func__, "Error sending angle to LED task, queue was full!");

        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
