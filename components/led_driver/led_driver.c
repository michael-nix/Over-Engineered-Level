#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"

#include "led_driver.h"
#include "ws2812.h"

static rmt_bytes_encoder_config_t led_bytes_encoder_config = {
    .bit0 = WS2818_BIT0,
    .bit1 = WS2818_BIT1,
    .flags.msb_first = 1,
};

static rmt_bytes_encoder_config_t led_reset_encoder_config = {
    .bit0 = WS2818_RESET_BIT,
    .bit1 = WS2818_RESET_BIT,
    .flags.msb_first = 0,
};

static rmt_channel_handle_t led_channel = NULL;
static rmt_encoder_handle_t led_encoder_handle = NULL;
static rmt_encoder_handle_t led_reset_handle = NULL;
static rmt_transmit_config_t led_tx_config = {.loop_count = 0};

esp_err_t led_initialize()
{
    rmt_tx_channel_config_t led_tx_channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = WS2812_DIN_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = (uint32_t)RMT_WS2812_TICK_RATE_HZ,
        .trans_queue_depth = 4,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&led_tx_channel_config, &led_channel));

    ESP_ERROR_CHECK(
        rmt_new_bytes_encoder(&led_bytes_encoder_config, &led_encoder_handle));

    ESP_ERROR_CHECK(
        rmt_new_bytes_encoder(&led_reset_encoder_config, &led_reset_handle));

    ESP_ERROR_CHECK(rmt_enable(led_channel));

    return ESP_OK;
}

esp_err_t led_set_pixels(LEDPixel* pixels, size_t npixels)
{
    uint8_t reset = 0;

    esp_err_t error = ESP_OK;
    error = rmt_transmit(led_channel, led_encoder_handle, pixels,
        sizeof(LEDPixel) * npixels, &led_tx_config);
    if (ESP_OK != error)
    {
        ESP_LOGE(__func__, "Failed to set pixel colours!");

        return error;
    }

    error =
        rmt_transmit(led_channel, led_reset_handle, &reset, 1, &led_tx_config);
    if (ESP_OK != error)
    {
        ESP_LOGE(__func__, "Failed to reset WS2818!");

        return error;
    }

    error = rmt_tx_wait_all_done(led_channel, -1);
    if (ESP_OK != error)
    {
        ESP_LOGE(
            __func__, "Failed to flush and wait for transactions to complete!");

        return error;
    }

    return ESP_OK;
}
