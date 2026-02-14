#pragma once

#include "mpu6050.h"

#include "driver/gpio.h"
#include "esp_err.h"

#if (DLPF_CFG_VAL == 0 || DLPF_CFG_VAL == 7)
#define IMU_SAMPLE_RATE_HZ (8000 / (1 + SMPLRT_DIV_VAL))
#else
#define IMU_SAMPLE_RATE_HZ (1000 / (1 + SMPLRT_DIV_VAL))
#endif

// how often (in ms) we want to get data from the IMU FIFO buffer.
#define IMU_PERIOD_MS 20

// amount of data, per sensor, that can be captured per IMU_PERIOD_MS.
#define IMU_SAMPLES_PER_PERIOD ((IMU_PERIOD_MS * IMU_SAMPLE_RATE_HZ) / 1000)

// number of sensors set to be catpured in the IMU FIFO buffer by the
// FIFO_EN_REG register.
#define IMU_NUM_SENSORS 6

// number of bytes per sensor measurement (always two for MPU6050)
#define IMU_NUM_BYTES_PER_MEASUREMENT 2

// number of bytes in the IMU FIFO buffer every IMU_PERIOD_MS
#define IMU_NUM_BUFFER_BYTES                                                   \
    (IMU_SAMPLES_PER_PERIOD * IMU_NUM_SENSORS * IMU_NUM_BYTES_PER_MEASUREMENT)

// number of individual data samples in the IMU FIFO buffer every IMU_PERIOD_MS
#define NUM_SAMPLES (IMU_SAMPLES_PER_PERIOD * IMU_NUM_SENSORS)

#define ACCELEROMETER_SENSITIVITY (16384.0f / (1 << AFS_SEL_VAL))

#define GYROSCOPE_SENSITIVITY (131.072f / (1 << FS_SEL_VAL))

typedef struct imu_data
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} IMUData;

/* `imu_initialize` - initializes the MPU6050 IMU, and configures an
   optional interrupt handler.

   Configures the MPU6050 IMU with the settings in the `mpu6050config` array of
   register-value pairs.If `interrupt_handler` is NULL, no interrupt handler
   will be configured.

   #### Parameters:
    - `interrupt_handler` - a GPIO ISR handler function to be called when the
   IMU interrupt pin is triggered,
    - `args` - arguments to be passed to the `interrupt_handler` when called.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the IMU could not be initialized,
    - `ESP_OK` if the IMU was successfully initialized.

   NOTE: will reset the system if communication with the IMU fails or
   intialization of GPIO fails.
*/
esp_err_t imu_initialize(gpio_isr_t interrupt_handler, void* args);

/* `imu_number_of_bytes_in_buffer` - reads the number of bytes currently in the
   IMU FIFO buffer.

   #### Parameters:
    - `nbytes` - pointer to a `uint16_t` where the number of bytes in the IMU
   FIFO buffer will be stored.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the IMU has not been initialized,
    - `ESP_OK` if the number of bytes was successfully read,
    - Any error generated when trying to read from the IMU.
*/
esp_err_t imu_number_of_bytes_in_buffer(uint16_t* nbytes);

/* `imu_get_data_from_buffer` - reads `nbytes` of data from the IMU FIFO buffer
   into `buffer`.

   #### Parameters:
    - `buffer` - pointer to a `uint8_t` array where the data will be stored,
    - `nbytes` - number of bytes to read from the IMU FIFO buffer.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the IMU has not been initialized,
    - `ESP_OK` if the data was successfully read,
    - Any error generated when trying to read from the IMU.
*/
esp_err_t imu_get_data_from_buffer(uint8_t* buffer, uint16_t nbytes);

/* `imu_reset_buffer` - resets the IMU FIFO buffer.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the IMU has not been initialized,
    - `ESP_OK` if the IMU FIFO buffer was successfully reset,
    - Any error generated when trying to write to the IMU.
*/
esp_err_t imu_reset_buffer(void);

/* `imu_reset_device` - resets the IMU.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if the IMU has not been initialized,
    - `ESP_OK` if the IMU was successfully reset.

   NOTE: will reset the system if it can't communicate with the IMU.
*/
esp_err_t imu_reset_device(void);
