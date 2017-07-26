/*
This software is subject to the license described in the license.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

/**@file
 * @brief The main file for the ANT Scan and Forward demo.
 *
 * @defgroup ant_scan_and_forward_example ANT Scan and Forward Demo
 * @{
 * @ingroup nrf_ant_scan_and_forward
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "nrf.h"
#include "bsp.h"
#include "boards.h"
#include "hardfault.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_timer.h"
#include "commands.h"
#include "scan_and_forward.h"
#include "ant_parameters.h"
#include "softdevice_handler.h"
#include "ant_stack_config.h"

#define APP_TIMER_PRESCALER           0x00      /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE       0x04      /**< Size of timer operation queues. */

/**@brief Function for dispatching a ANT stack event to all modules with a ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after a ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
    switch (p_ant_evt->channel)
    {
        case SF_ANT_BS_CHANNEL_NUMBER:
            sf_background_scanner_process(p_ant_evt);
            break;

        case SF_ANT_MS_CHANNEL_NUMBER:
            sf_master_beacon_process(p_ant_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
    uint32_t err_code;

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        sf_bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
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


/**@brief Initializes the battery board switches as inputs.
 */
void switch_init(void)
{
#if defined(BSP_SWITCH_0) && defined(BSP_SWITCH_1) && defined(BSP_SWITCH_2) && \
    defined(BSP_SWITCH_3) && defined(BSP_SWITCH_4) // Only initialize switches on boards which have switches defined
    nrf_gpio_cfg_input(BSP_SWITCH_0, SWITCH_PULL);
    nrf_gpio_cfg_input(BSP_SWITCH_1, SWITCH_PULL);
    nrf_gpio_cfg_input(BSP_SWITCH_2, SWITCH_PULL);
    nrf_gpio_cfg_input(BSP_SWITCH_3, SWITCH_PULL);
    nrf_gpio_cfg_input(BSP_SWITCH_4, SWITCH_PULL);
#endif
}


/** @brief The main function
 */
int main(void)
{
    uint32_t err_code;

    switch_init();
    utils_setup();
    softdevice_setup();

    sf_init();

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
