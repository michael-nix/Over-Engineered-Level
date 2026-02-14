#pragma once

#include "esp_err.h"

enum imu_events
{
    IMU_BUFFER_FULL = BIT0,
    IMU_TIMER_EXPIRED = BIT1,

    // All bits set: must be one more bit than final event, minus one.
    IMU_ALL_EVENTS = (BIT2 - 1),
};

/* `imu_task` - task that manages the MPU6050 IMU, and captures data from it.

   This task must be created after the IMU driver has been initialized.

   #### Parameters:
    - `pvParameters` - a pointer to an `IntervalBuffer` that holds IMU data.

   NOTE: the input parameter `IntervalBuffer` is shared with the data task, so
   locks or notifications must be used to ensure mutual exclusion.
*/
void imu_task(void* pvParameters);

/* `imu_interrupt_handler` - GPIO ISR handler for the IMU interrupt pin.

   This function should be passed as the `interrupt_handler` parameter to
   `imu_initialize` when initializing the IMU driver.

   #### Parameters:
    - `args` - a pointer to the arguments that will be passed to the ISR handler
   when called.
*/
void imu_interrupt_handler(void* args);

/* `imu_send_event` - used by timers and interrupts to send events to the IMU
   task to notify it when IMU data is ready, or when the FIFO buffer has
   overflowed.

   #### Parameters:
    - `event` - the event to send to the `imu_task`, as defined in `enum
   imu_events`.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the `imu_task` handle has not been set,
    - `ESP_OK` if the event was successfully sent.
*/
esp_err_t imu_send_event(enum imu_events);

/* `imu_task_notify` - send a notification to the `imu_task`, primarily when the
   shared `IntervalBuffer` is free for use.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the `imu_task` handle has not been set,
    - `ESP_OK` if the notification was successfully sent.
*/
esp_err_t imu_task_notify();
