/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/**
 * This file contains the source code for a sample application using both Nordic Gazell proprietary
 * radio protocol and Bluetooth Low Energy radio protocol. In Bluetooth mode, it behave as a Heart
 * Rate sensor, in Gazell mode it behaves as a 'device'.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble_app_gzll_device.h"
#include "ble_app_gzll_hr.h"
#include "ble_app_gzll_ui.h"
#include "ble_app_gzll_common.h"
#include "app_timer.h"
#include "bsp.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// Store current mode
volatile radio_mode_t running_mode = BLE;

/**
 * @brief Function for the Power Management.
 */
static void power_manage(void)
{
    if (running_mode == GAZELL)
    {
        // Use directly __WFE and __SEV macros since the SoftDevice is not available in proprietary
        // mode.
        // Wait for event.
        __WFE();

        // Clear Event Register.
        __SEV();
        __WFE();
    }
    else if (running_mode == BLE)
    {
        uint32_t err_code;

        // Use SoftDevice API for power_management when in Bluetooth Mode.
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t     err_code;
    radio_mode_t previous_mode = running_mode;

    // Setup logger.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Multiprotocol application example\r\n");
    NRF_LOG_INFO("BLE mode\r\n");

    ble_stack_start();
    NRF_LOG_FLUSH();

    // Initialize timer module, making it use the scheduler.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    bsp_init_app();
    ble_hrs_app_start();

    // Enter main loop.
    for (;;)
    {
        bool gzll_button;
        bool ble_button;

        // Check buttons states.
        err_code = bsp_button_is_pressed(BLE_BUTTON_ID, &ble_button);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_button_is_pressed(GZLL_BUTTON_ID, &gzll_button);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_FLUSH();
        power_manage();

        if (ble_button)
        {
            running_mode = BLE;
        }
        else if (gzll_button)
        {
            running_mode = GAZELL;
        }

        if (running_mode != previous_mode)
        {
            previous_mode = running_mode;
            if (running_mode == GAZELL)
            {
                NRF_LOG_INFO("Gazell mode\r\n");

                // Stop all heart rate functionality before disabling the SoftDevice.
                ble_hrs_app_stop();

                // Disable the softdevice stack.
                ble_stack_stop();

                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                APP_ERROR_CHECK(err_code);

                // Enable Gazell.
                gzll_app_start();
            }
            else if (running_mode == BLE)
            {
                NRF_LOG_INFO("BLE mode\r\n");

                // Disable Gazell.
                gzll_app_stop();

                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                APP_ERROR_CHECK(err_code);

                // Re-enable the softdevice stack.
                ble_stack_start();
                ble_hrs_app_start();
            }
        }
    }
}


