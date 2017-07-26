/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "boards.h"
#include "hardfault.h"
#include "softdevice_handler.h"
#include "ant_stack_config.h"
#include "ant_search_sharing.h"


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    uint32_t           err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Setup SoftDevice and events handler
    err_code = softdevice_ant_evt_handler_set(ant_search_sharing_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_handler_init(&clock_lf_cfg, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);

    // Initialize LEDs
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    // Setup search sharing and start channels
    ant_search_sharing_setup();

    // Enter main loop
    for (;;)
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


