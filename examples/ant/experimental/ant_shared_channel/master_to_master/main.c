/*
This software is subject to the license described in the license.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/

/**@file
 * @defgroup ant_shared_channel_master_demo ANT Auto Shared Master Example
 * @{
 * @ingroup ant_shared_channel
 *
 * @brief Example of ANT Auto Shared Channel (ASC) Master.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

// Version 0.0.2


#include <stdint.h>
#include "asc_coordinator.h"
#include "boards.h"
#include "hardfault.h"
#include "n5sk_led.h"
#include "nrf_delay.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ant_error.h"
#include "leds.h"

    #define LED_ERROR_0     BSP_LED_0
    #define LED_ERROR_1     BSP_LED_1
    #define LED_ERROR_AUX   BSP_LED_0

    #define LED_START_UP    BSP_LED_0


#ifndef BLE_STACK_SUPPORT_REQD
/**@brief Function for stack interrupt handling.
 *
 * Implemented to clear the pending flag when receiving
 * an interrupt from the stack.
 */
void SD_EVT_IRQHandler(void)
{
}
#endif

/**@brief Function for handling SoftDevice asserts.
 *
 * @param[in] pc          Value of the program counter.
 * @param[in] line_num    Line number where the assert occurred.
 * @param[in] p_file_name Pointer to the file name.
 */
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t * p_file_name)
{
    for (;;)
    {
        // No implementation needed.
    }
}


/**@brief Function for handling HardFault.
 */
void HardFault_process(HardFault_stack_t *p_stack)
{
    for (;;)
    {
        // No implementation needed.
        #ifdef DEBUG_LED
            led_on(LED_ERROR_0);
            nrf_delay_ms(20);
            led_off(LED_ERROR_0);
        #endif
    }
}


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    uint32_t err_code;

    // Configure pins LED_A - LED_D as outputs.
    led_init();

    // Turn LED_A on to indicate that the application is running.
    led_on(LED_START_UP);

    ascc_init();

    // Turn LED_A off to indicate that stack is enabled.
    led_off(LED_START_UP);

    // Main loop.
    for (;;)
    {
        // Put CPU in sleep if possible.
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);

        ascc_poll_for_ant_evets();
    }
}


/**
 *@}
 **/
