/*
This software is subject to the license described in the license.txt file included with
this software distribution.
You may not use this file except in compliance with this license.

Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

/*
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "bsp.h"
#include "boards.h"
#include "hardfault.h"
#include "softdevice_handler.h"
#include "ant_stack_config.h"
#include "ant_advanced_burst.h"

#define APP_TIMER_OP_QUEUE_SIZE  0x04  ///< Size of timer operation queues.
#define APP_TIMER_PRESCALER      0x00  ///< Value of the RTC1 PRESCALER register.

/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
    uint32_t err_code;

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        ant_advanced_burst_bsp_evt_handler);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Setup buttons and timer
    utils_setup();

    // Initialize LEDs
    LEDS_CONFIGURE(LEDS_MASK);

    // Setup SoftDevice and events handler
    err_code = softdevice_ant_evt_handler_set(ant_advanced_burst_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_handler_init(&clock_lf_cfg, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);

    // Setup advanced burst and start channel
    ant_advanced_burst_setup();

    // Enter main loop
    for (;;)
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


