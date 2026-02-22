#include "data_task.h"
#include "attitude.h"
#include "filters.h"
#include "imu_driver.h"
#include "imu_task.h"
#include "led_task.h"

#include <math.h>

#define GRAVITY 9.80665f

static TaskHandle_t data_task_handle = NULL;

static inline esp_err_t filter_IMUData(IMUData* new_data, IMUData* old_data,
    SecondOrderSection* sos, SOSData* sos_data, IMUData* average)
{
    if (NULL == new_data || NULL == old_data || NULL == sos ||
        NULL == sos_data || NULL == average)
    {
        ESP_LOGE(__func__, "Can't filter IMU data, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    static float dn = 1.0f / IMU_SAMPLES_PER_PERIOD;
    IMUData filtered[IMU_SAMPLES_PER_PERIOD] = {0};

    for (size_t sample = 0; sample < IMU_SAMPLES_PER_PERIOD; sample++)
    {
        for (size_t section = 0; section < FILTER_NUM_SOS_SECTIONS; section++)
        {
            filtered[sample].ax =
                filter_sos(&sos[section], &sos_data[0], new_data[sample].ax);
            filtered[sample].ay =
                filter_sos(&sos[section], &sos_data[1], new_data[sample].ay);
            filtered[sample].az =
                filter_sos(&sos[section], &sos_data[2], new_data[sample].az);

            filtered[sample].gx =
                filter_sos(&sos[section], &sos_data[3], new_data[sample].gx);
            filtered[sample].gy =
                filter_sos(&sos[section], &sos_data[4], new_data[sample].gy);
            filtered[sample].gz =
                filter_sos(&sos[section], &sos_data[5], new_data[sample].gz);
        }

        average->ax += filtered[sample].ax;
        average->ay += filtered[sample].ay;
        average->az += filtered[sample].az;

        average->gx += old_data[sample].gx - filtered[sample].gx;
        average->gy += old_data[sample].gy - filtered[sample].gy;
        average->gz += old_data[sample].gz - filtered[sample].gz;
    }

    average->ax *= dn;
    average->ay *= dn;
    average->az *= dn;

    average->gx *= dn;
    average->gy *= dn;
    average->gz *= dn;

    return ESP_OK;
}

void data_task(void* pvParameters)
{
    data_task_handle = xTaskGetCurrentTaskHandle();
    IntervalBuffer* imu_data = (IntervalBuffer*)pvParameters;

    SecondOrderSection sos[FILTER_NUM_SOS_SECTIONS];
    get_butter_sos(
        FILTER_CUTOFF_FREQUENCY_HZ, IMU_SAMPLE_RATE_HZ, FILTER_ORDER, sos);

    bool set_sos_data = true;
    SOSData sos_data[IMU_NUM_SENSORS] = {0};

    initialize_attitude_filters(
        1.0f / IMU_SAMPLE_RATE_HZ * IMU_SAMPLES_PER_PERIOD, 1.0f, 1.0f);
    float R[3][3] = EYE3;

    bool set_down_vector = true;
    float down[3] = {0};

    while (true)
    {
        // wait for `imu_data` to have new data
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

        IMUData new_data[IMU_SAMPLES_PER_PERIOD];
        get_current_interval_from_buffer(imu_data, (uint8_t*)new_data);

        IMUData old_data[IMU_SAMPLES_PER_PERIOD];
        pop_interval_from_buffer_with_delay(
            imu_data, (uint8_t*)old_data, FILTER_GROUP_DELAY_SAMPLES);

        // let `imu_task` know we no longer need `imu_data`
        imu_task_notify();

        if (set_sos_data)
        {
            initialize_sos_data(
                sos_data, (float*)&new_data[0], IMU_NUM_SENSORS);

            set_sos_data = false;
        }

        IMUData average = {0};
        filter_IMUData(new_data, old_data, sos, sos_data, &average);

        // set initial down vector:
        if (set_down_vector)
        {
            down[0] = average.ax;
            down[1] = average.ay;
            down[2] = average.az;

            float dmag = sqrtf(
                down[0] * down[0] + down[1] * down[1] + down[2] * down[2]);
            down[0] /= dmag;
            down[1] /= dmag;
            down[2] /= dmag;

            set_down_vector = false;
        }

        // estimate current attitude:
        float a[3] = {average.ax, average.ay, average.az};
        float w[3] = {average.gx, average.gy, average.gz};
        mdm_filter(R, down, a, w);

        // get original down vector in current reference frame:
        float r[3] = {0};
        get_current_direction(R, down, r);

        float intensity = sqrtf(r[0] * r[0] + r[1] * r[1]);

        // the LEDs are aligned with -y as its x-axis, and -x as its y-axis.
        float angle = atan2f(-r[0], -r[1]);

        LEDData data = {.angle = angle, .intensity = intensity};
        send_data_to_led_task(&data);
    }
}

esp_err_t data_task_notify()
{
    if (NULL == data_task_handle)
    {
        ESP_LOGE(__func__, "Can't signal data task, handle is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    // eNoAction always returns pdPASS
    xTaskNotify(data_task_handle, 0, eNoAction);

    return ESP_OK;
}
