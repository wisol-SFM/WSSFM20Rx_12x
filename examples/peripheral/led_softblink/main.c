/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup led_softblink_example_main main.c
 * @{
 * @ingroup led_softblink_example
 * @brief LED Soft Blink Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "led_softblink.h"
#include "app_error.h"
#include "sdk_errors.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "app_util_platform.h"

/*Timer initalization parameters*/
#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     0

/**
 * @brief Function for starting lfclk needed by APP_TIMER.
 */
static void lfclk_init(void)
{
    uint32_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;

    lfclk_init();

    // Start APP_TIMER to generate timeouts.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, OP_QUEUES_SIZE, NULL);

    led_sb_init_params_t led_sb_init_param = LED_SB_INIT_DEFAULT_PARAMS(LEDS_MASK);

    err_code = led_softblink_init(&led_sb_init_param);
    APP_ERROR_CHECK(err_code);

    err_code = led_softblink_start(LEDS_MASK);
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        __WFE();
    }
}

/** @} */
