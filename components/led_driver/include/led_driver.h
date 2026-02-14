#pragma once

typedef struct pixel
{
    uint8_t g;
    uint8_t r;
    uint8_t b;
} LEDPixel;

/* `led_initialize` - initializes the LED driver for the WS-2812.

   #### Returns:
    - `ESP_OK` if the LED driver was successfully initialized, resets the system
   if there is an error itializing the driver.
*/
esp_err_t led_initialize();

/* `led_set_pixels` - sets the colours of the WS-2812 LEDs.

   #### Parameters:
    - `pixels` - pointer to an array of `LEDPixel`s that define the colours of
   each LED,
    - `npixels` - number of pixels in the `pixels` array.

   #### Returns:
    - `ESP_OK` if the pixel colours were successfully set,
    - Any error generated when trying to set the pixel colours.
*/
esp_err_t led_set_pixels(LEDPixel* pixels, size_t npixels);
