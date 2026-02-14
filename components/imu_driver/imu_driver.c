#include "imu_driver.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define NCONFIG 9
static const uint8_t mpu6050config[NCONFIG][2] = {
    {PWR_MGMT_1_REG, CLKSEL_VAL}, // Set clock to gyro-x
    {SMPRT_DIV_REG,
        SMPLRT_DIV_VAL},        // Set sample rate to 1 kHz if DLPF is enabled
    {CONFIG_REG, DLPF_CFG_VAL}, // Enable highest bandwidth DLPF
    {GYRO_CONFIG_REG, FS_SEL_BIT}, // Set gyro range +/- 500 deg/s
    {ACCEL_CONFIG_REG, ACCEL_HPF_VAL | AFS_SEL_BIT}, // Set accel range +/- 2g
    {FIFO_EN_REG, XG_FIFO_EN_BIT | YG_FIFO_EN_BIT | ZG_FIFO_EN_BIT |
                      ACCEL_FIFO_EN_BIT}, // Enable FIFO for gyro and accel
    {INT_PIN_CFG_REG, 0x00},              // Configure interrupt pin
    {INT_ENABLE_REG, FIFO_OFLOW_EN_BIT},  // Enable FIFO overflow interrupt
    {USER_CTRL_REG, FIFO_EN_BIT},         // Reset FIFO and enable
};

static inline esp_err_t mpu6050_write_register(
    i2c_master_dev_handle_t handle, uint8_t mpu_register, uint8_t mpu_data);

static inline esp_err_t mpu6050_write_register_no_check(
    i2c_master_dev_handle_t handle, uint8_t mpu_register, uint8_t mpu_data);

static inline esp_err_t mpu6050_read_register(i2c_master_dev_handle_t handle,
    uint8_t mpu_register, uint8_t* buffer, size_t size);

static inline void imu_configure_device();

static i2c_master_bus_handle_t master_bus_handle = NULL;
static i2c_master_dev_handle_t mpu6050_master_handle = NULL;

esp_err_t imu_initialize(gpio_isr_t interrupt_handler, void* args)
{
    i2c_master_bus_config_t master_bus_config = {
        .i2c_port = -1,
        .scl_io_num = MPU6050_SCL_PIN,
        .sda_io_num = MPU6050_SDA_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = false,
        .flags.allow_pd = false,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&master_bus_config, &master_bus_handle));
    ESP_LOGI(__func__, "I2C master bus configured.");

    i2c_device_config_t mpu6050_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDRESS,
        .scl_speed_hz = MPU6050_SCL_SPEED_HZ,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        master_bus_handle, &mpu6050_config, &mpu6050_master_handle));
    ESP_LOGI(__func__, "I2C master device added to bus.");

    imu_configure_device();
    ESP_LOGI(__func__, "IMU successfully configured.");

    if (NULL == interrupt_handler)
        return ESP_OK;

    gpio_config_t i2c_interrupt = {
        .intr_type = MPU6050_INTR_TYPE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1 << MPU6050_INT_PIN,
        .pull_down_en = MPU6050_PULLDOWN_EN,
        .pull_up_en = MPU6050_PULLUP_EN,
    };

    ESP_ERROR_CHECK(gpio_config(&i2c_interrupt));
    ESP_LOGI(__func__, "GPIO successfully configured.");

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_LOGI(__func__, "GPIO service installed.");

    ESP_ERROR_CHECK(
        gpio_isr_handler_add(MPU6050_INT_PIN, interrupt_handler, args));
    ESP_LOGI(__func__, "GPIO ISR handler added.");

    return ESP_OK;
}

static inline esp_err_t mpu6050_read_register(i2c_master_dev_handle_t handle,
    uint8_t mpu_register, uint8_t* buffer, size_t size)
{
    esp_err_t error = i2c_master_transmit_receive(
        handle, &mpu_register, sizeof(mpu_register), buffer, size, -1);

    return error;
}

static inline esp_err_t mpu6050_write_register_no_check(
    i2c_master_dev_handle_t handle, uint8_t mpu_register, uint8_t mpu_data)
{
    uint8_t command[2] = {mpu_register, mpu_data};
    esp_err_t error = i2c_master_transmit(handle, command, sizeof(command), -1);
    if (error != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to write register.");

        return error;
    }

    return ESP_OK;
}

static inline esp_err_t mpu6050_write_register(
    i2c_master_dev_handle_t handle, uint8_t mpu_register, uint8_t mpu_data)
{
    uint8_t command[2] = {mpu_register, mpu_data};
    esp_err_t error = i2c_master_transmit(handle, command, sizeof(command), -1);
    if (error != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to write register.");

        return error;
    }

    uint8_t value;
    error = mpu6050_read_register(handle, mpu_register, &value, 1);
    if (error != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to read back register.");

        return error;
    }

    if (value != mpu_data)
    {
        ESP_LOGE(__func__,
            "Failed to read back register, values don't match.  Got 0x%x, "
            "expected 0x%x",
            value, mpu_data);

        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t imu_reset_device(void)
{
    // reset the device:
    ESP_ERROR_CHECK(mpu6050_write_register_no_check(
        mpu6050_master_handle, PWR_MGMT_1_REG, DEVICE_RESET_BIT));
    vTaskDelay(pdMS_TO_TICKS(100));

    // reset the signal paths:
    ESP_ERROR_CHECK(mpu6050_write_register_no_check(mpu6050_master_handle,
        SIGNAL_PATH_RESET_REG,
        GYRO_RESET_BIT | ACCEL_RESET_BIT | TEMP_RESET_BIT));
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

static inline void imu_configure_device()
{
    if (NULL == mpu6050_master_handle)
    {
        ESP_LOGE(__func__, "Master device handle has not been initialized!");
    }

    imu_reset_device();

    for (size_t idx = 0; idx < NCONFIG; idx++)
    {
        ESP_ERROR_CHECK(mpu6050_write_register(mpu6050_master_handle,
            mpu6050config[idx][0], mpu6050config[idx][1]));
    }
}

esp_err_t imu_number_of_bytes_in_buffer(uint16_t* nbytes)
{
    if (NULL == mpu6050_master_handle)
    {
        ESP_LOGE(__func__, "Master device handle has not been initialized!");

        return ESP_ERR_INVALID_ARG;
    }

    uint8_t count[2] = {0};
    esp_err_t error = mpu6050_read_register(
        mpu6050_master_handle, FIFO_COUNT_H_REG, count, sizeof(count));
    if (ESP_OK != error)
    {
        ESP_LOGW(__func__, "Error reading IMU buffer.");
        *nbytes = 0;

        return error;
    }

    *nbytes = (uint16_t)(count[0] << 8 | count[1]);

    return ESP_OK;
}

esp_err_t imu_get_data_from_buffer(uint8_t* buffer, uint16_t nbytes)
{
    if (NULL == mpu6050_master_handle)
    {
        ESP_LOGE(__func__, "Master device handle has not been initialized!");

        return ESP_ERR_INVALID_ARG;
    }

    return mpu6050_read_register(
        mpu6050_master_handle, FIFO_R_W_REG, buffer, nbytes);
}

esp_err_t imu_reset_buffer(void)
{
    if (NULL == mpu6050_master_handle)
    {
        ESP_LOGE(__func__, "Master device handle has not been initialized!");

        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = mpu6050_write_register_no_check(
        mpu6050_master_handle, USER_CTRL_REG, FIFO_RESET_BIT);
    if (ESP_OK != error)
    {
        ESP_LOGE(__func__, "Failed to disable IMU buffer.");

        return error;
    }

    error = mpu6050_write_register(
        mpu6050_master_handle, USER_CTRL_REG, FIFO_EN_BIT);
    if (ESP_OK != error)
    {
        ESP_LOGE(__func__, "failed to enable IMU buffer.");

        return error;
    }

    return ESP_OK;
}
