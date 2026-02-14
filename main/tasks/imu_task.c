#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "data_task.h"
#include "filters.h"
#include "imu_driver.h"
#include "imu_task.h"

#include <math.h>

static EventGroupHandle_t imu_events;
static TaskHandle_t imu_task_handle;

esp_err_t imu_send_event(enum imu_events event)
{
    if (NULL == imu_events)
    {
        ESP_LOGE(
            __func__, "Can't send event to IMU task, event group is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    xEventGroupSetBits(imu_events, event);

    return ESP_OK;
}

esp_err_t imu_task_notify()
{
    if (NULL == imu_task_handle)
    {
        ESP_LOGE(__func__, "Can't signal IMU task, handle is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    // eNoAction always returns pdPASS
    xTaskNotify(imu_task_handle, 0, eNoAction);

    return ESP_OK;
}

void imu_interrupt_handler(void* args)
{
    BaseType_t task_woken = pdFALSE;
    BaseType_t result =
        xEventGroupSetBitsFromISR(imu_events, IMU_BUFFER_FULL, &task_woken);

    if (pdPASS == result)
    {
        portYIELD_FROM_ISR(task_woken);
    }
}

static void imu_timer_callback(TimerHandle_t timer)
{
    xEventGroupSetBits(imu_events, IMU_TIMER_EXPIRED);
}

static inline float convert_accel(uint8_t* data)
{
    return (int16_t)(data[0] << 8 | data[1]) / ACCELEROMETER_SENSITIVITY *
           9.80665;
}

static inline float convert_gyro(uint8_t* data)
{
    return (int16_t)(data[0] << 8 | data[1]) / GYROSCOPE_SENSITIVITY / 180 *
           3.14159;
}

void imu_task(void* pvParameters)
{
    IntervalBuffer* imu_data = (IntervalBuffer*)pvParameters;

    imu_task_handle = xTaskGetCurrentTaskHandle();
    imu_events = xEventGroupCreate();
    uint8_t buffer[IMU_NUM_BUFFER_BYTES] = {0};

    // ESP high resolution timer makes no difference in over/underflow.
    TimerHandle_t mpu_timer = xTimerCreate("imu_timer",
        pdMS_TO_TICKS(IMU_PERIOD_MS), pdTRUE, NULL, imu_timer_callback);

    // let the IMU stabilize, then reset buffer & flags so operation can begin:
    vTaskDelay(pdMS_TO_TICKS(100));
    imu_reset_buffer();
    xEventGroupClearBits(imu_events, IMU_BUFFER_FULL);

    xTimerStart(mpu_timer, 0);

    esp_err_t error = ESP_OK;
    while (true)
    {
        EventBits_t imu_bits = xEventGroupWaitBits(
            imu_events, IMU_ALL_EVENTS, pdTRUE, pdFALSE, portMAX_DELAY);

        if ((imu_bits & IMU_BUFFER_FULL) != 0)
        {
            ESP_LOGE(__func__, "IMU buffer is full! Resetting buffer.");
            imu_reset_buffer();
            xTimerReset(mpu_timer, 0);

            continue;
        }

        uint16_t ndata = 0;
        uint16_t nmissed = 0;
        error = imu_number_of_bytes_in_buffer(&ndata);
        if (error != ESP_OK)
        {
            ESP_LOGE(__func__, "Error reading FIFO buffer, resetting.");
            imu_reset_buffer();
            xTimerReset(mpu_timer, 0);

            continue;
        }

        if (IMU_NUM_BUFFER_BYTES < ndata)
        {
            ESP_LOGD(__func__,
                "Too many bytes in buffer! Got %d instead of %d.", ndata,
                IMU_NUM_BUFFER_BYTES);

            // this is ok, just grab the right amount
            ndata = IMU_NUM_BUFFER_BYTES;
        }

        error = imu_get_data_from_buffer(buffer, ndata);
        if (error != ESP_OK)
        {
            ESP_LOGE(__func__, "Error reading FIFO buffer, resetting.");
            imu_reset_buffer();
            xTimerReset(mpu_timer, 0);

            continue;
        }

        if (IMU_NUM_BUFFER_BYTES > ndata)
        {
            ESP_LOGD(__func__,
                "Expected number of bytes doesn't match! Got %d instead of %d.",
                ndata, IMU_NUM_BUFFER_BYTES);

            // this is ok, just copy the last good full values
            ndata = ndata -
                    ndata % (IMU_NUM_SENSORS * IMU_NUM_BYTES_PER_MEASUREMENT);
            nmissed = (uint16_t)ceil(
                (float)(IMU_NUM_BUFFER_BYTES - ndata) /
                (IMU_NUM_SENSORS * IMU_NUM_BYTES_PER_MEASUREMENT));
        }
        else if (IMU_NUM_BUFFER_BYTES < ndata)
        {
            ESP_LOGI(__func__, "resetting IMU buffer.");
            // We can't read the full FIFO, so it won't reset, so manual reset.
            // We will lose some data, but better than losing a full period, and
            // gaps will be filled in with copies in the next period.
            imu_reset_buffer();
        }

        size_t bytes_idx = 0;
        IMUData data = {0};
        for (size_t data_idx = 0; data_idx < IMU_SAMPLES_PER_PERIOD; data_idx++)
        {
            data.ax = convert_accel(&buffer[bytes_idx]);
            bytes_idx += 2;

            data.ay = convert_accel(&buffer[bytes_idx]);
            bytes_idx += 2;

            data.az = convert_accel(&buffer[bytes_idx]);
            bytes_idx += 2;

            data.gx = convert_gyro(&buffer[bytes_idx]);
            bytes_idx += 2;

            data.gy = convert_gyro(&buffer[bytes_idx]);
            bytes_idx += 2;

            data.gz = convert_gyro(&buffer[bytes_idx]);
            bytes_idx += 2;

            add_item_to_buffer(imu_data, (uint8_t*)&data);

            if (bytes_idx == ndata)
            {
                break;
            }
        }

        for (size_t idx = 0; idx < nmissed; idx++)
        {
            add_item_to_buffer(imu_data, (uint8_t*)&data);
        }

        if (imu_data->ready)
        {
            data_task_notify();

            xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        }
    }
}
