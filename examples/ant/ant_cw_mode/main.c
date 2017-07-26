/*
   This software is subject to the license described in the license.txt file
   included with this software distribution. You may not use this file except in compliance
   with this license.

   Copyright (c) Dynastream Innovations Inc. 2016
   All rights reserved.
 */

/**@file
 * @defgroup nrf_ant_background_scanning_demo ANT CW Mode Example
 * @{
 * @ingroup nrf_ant_cw_mode_demo
 *
 * @brief Example of ANT CW Mode Implementation
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S332 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

#include "app_error.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_stack_config.h"
#include "boards.h"
#include "hardfault.h"
#include "softdevice_handler.h"
#include "sdk_config.h"


/**@brief Initialize application.
 */
static void application_initialize()
{
    uint32_t err_code;

    err_code = sd_ant_cw_test_mode_init();
    APP_ERROR_CHECK(err_code);

    // sd_ant_cw_test_mode assumes that the hfclck is enabled.
    // Request and wait for it to be ready.
    err_code = sd_clock_hfclk_request();
    APP_ERROR_CHECK(err_code);

    uint32_t hfclk_is_running = 0;

    while (!hfclk_is_running)
    {
        APP_ERROR_CHECK(sd_clock_hfclk_is_running(&hfclk_is_running) );
    }

    // CW Mode at +4dBm, 2410 MHz with Modulated Transmission
    err_code = sd_ant_cw_test_mode(RADIO_FREQ_OFFSET,
                                   RADIO_TX_POWER_LVL_CUSTOM,
                                   RADIO_TXPOWER_TXPOWER_Pos4dBm,
                                   MODULATED_TRANSMISSION_TEST_MODE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    err_code = softdevice_handler_init(&clock_lf_cfg, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);
}


/* Main function */
int main(void)
{
    ret_code_t err_code;

    // Blink LED after successful initialization.
    LEDS_CONFIGURE(BSP_LED_0_MASK);

    softdevice_setup();
    application_initialize();

    LEDS_OFF(BSP_LED_0_MASK);

    // Enter main loop
    for (;;)
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


/**
 *@}
 **/
