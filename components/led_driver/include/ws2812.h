#pragma once

/*

Defines the RMT tick rate, pulse widths, and bits used by the RMT driver to
communicate with WS-2812 LEDs.

*/

#define RMT_WS2812_TICK_RATE_HZ 10e6
#define RMT_WS2812_T0H_TICKS    (uint32_t)(300e-9 * RMT_WS2812_TICK_RATE_HZ)
#define RMT_WS2812_T0L_TICKS    (uint32_t)(700e-9 * RMT_WS2812_TICK_RATE_HZ)
#define RMT_WS2812_T1H_TICKS    (uint32_t)(800e-9 * RMT_WS2812_TICK_RATE_HZ)
#define RMT_WS2812_T1L_TICKS    (uint32_t)(600e-9 * RMT_WS2812_TICK_RATE_HZ)
#define RMT_WS2812_RES_TICKS    (uint32_t)(50e-6 * RMT_WS2812_TICK_RATE_HZ / 2)

#define WS2818_NUM_PIXELS 8

#define WS2818_BIT0                                                            \
    {                                                                          \
        .level0 = 1,                                                           \
        .duration0 = RMT_WS2812_T0H_TICKS,                                     \
        .level1 = 0,                                                           \
        .duration1 = RMT_WS2812_T0L_TICKS,                                     \
    }

#define WS2818_BIT1                                                            \
    {                                                                          \
        .level0 = 1,                                                           \
        .duration0 = RMT_WS2812_T1H_TICKS,                                     \
        .level1 = 0,                                                           \
        .duration1 = RMT_WS2812_T1L_TICKS,                                     \
    }

#define WS2818_RESET_BIT                                                       \
    {                                                                          \
        .level0 = 0,                                                           \
        .duration0 = RMT_WS2812_RES_TICKS,                                     \
        .level1 = 0,                                                           \
        .duration1 = RMT_WS2812_RES_TICKS,                                     \
    }
