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
#include "app_error.h"
#include "boards.h"
#include "hardfault.h"
#include "softdevice_handler.h"
#include "ant_stack_config.h"
#include "ant_multi_channels_encrypted_rx.h"


/**@brief Function for dispatching an ANT stack event to all modules with an ANT stack event handler.
 *
 * @details This function is called from the ANT stack event interrupt handler after an ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
    ant_se_event_handler(p_ant_evt);
}


/**@brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_handler_init(&clock_lf_cfg, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for setup thinks not directly related to ANT processing
 *
 * Initialize LEDs
 */
static void utils_setup(void)
{
    // Initialize LEDs
    LEDS_CONFIGURE(LEDS_MASK);

    // display state of zero count of encrypted channel received.
    ant_se_num_of_decrypted_channels_display();
}


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    uint32_t err_code;

    softdevice_setup();

    utils_setup();

    // Setup as an RX Slave.
    ant_se_channel_rx_broadcast_setup();

    // Main loop.
    for (;;)
    {
        // Put CPU in sleep if possible
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


