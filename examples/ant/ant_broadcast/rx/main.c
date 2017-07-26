/*
This software is subject to the license described in the license.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/

/**@file
 * @defgroup ant_broadcast_rx_example ANT Broadcast RX Example
 * @{
 * @ingroup nrf_ant_broadcast
 *
 * @brief Example of basic ANT Broadcast RX.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "nrf.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_timer.h"
#include "bsp.h"
#include "hardfault.h"
#include "nordic_common.h"
#include "ant_stack_config.h"
#include "ant_channel_config.h"
#include "softdevice_handler.h"

#define APP_TIMER_PRESCALER             0x00    /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         0x04    /**< Size of timer operation queues. */

// Channel configuration.
#define ANT_BROADCAST_CHANNEL_NUMBER    0x00    /**< ANT Channel 0. */
#define EXT_ASSIGN_NONE                 0x00    /**< ANT Ext Assign. */

// Miscellaneous defines.
#define ANT_NETWORK_NUMBER              0x00    /**< Default public network number. */


/**@brief Function for dispatching a ANT stack event to all modules with an ANT stack event handler.
 *
 * @details This function is called from the ANT stack event interrupt handler after an ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
    uint32_t err_code;

    if (p_ant_evt->channel == ANT_BROADCAST_CHANNEL_NUMBER)
    {
        ANT_MESSAGE * p_message = (ANT_MESSAGE *)p_ant_evt->msg.evt_buffer;

        switch (p_ant_evt->event)
        {
            case EVENT_RX:

                if (p_message->ANT_MESSAGE_ucMesgID == MESG_BROADCAST_DATA_ID
                    || p_message->ANT_MESSAGE_ucMesgID == MESG_ACKNOWLEDGED_DATA_ID
                    || p_message->ANT_MESSAGE_ucMesgID == MESG_BURST_DATA_ID)
                {
                    err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
                    APP_ERROR_CHECK(err_code);
                }
                break;

            default:
                break;
        }
    }
}


/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
    uint32_t err_code;

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = bsp_init(BSP_INIT_LED,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        NULL);
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


/**@brief Function for setting up the ANT module to be ready for RX broadcast.
 */
static void ant_channel_rx_broadcast_setup(void)
{
    uint32_t err_code;

    ant_channel_config_t broadcast_channel_config =
    {
        .channel_number    = ANT_BROADCAST_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_SLAVE,
        .ext_assign        = EXT_ASSIGN_NONE,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .device_number     = CHAN_ID_DEV_NUM,
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_NETWORK_NUMBER,
    };

    err_code = ant_channel_init(&broadcast_channel_config);
    APP_ERROR_CHECK(err_code);

    // Open channel.
    err_code = sd_ant_channel_open(ANT_BROADCAST_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    utils_setup();
    softdevice_setup();
    ant_channel_rx_broadcast_setup();

    // Main loop.
    for (;;)
    {
#if CPU_LOAD_TRACE
        // Disabling interrupts in this way is highly not recommended. It has an impact on the work
        // of the SoftDevice and is used only to show CPU load.
        __disable_irq();
        LEDS_OFF(BSP_LED_0_MASK);
        __WFI();
        LEDS_ON(BSP_LED_0_MASK);
        __enable_irq();
#else
        // Put CPU in sleep if possible.
        uint32_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
#endif // CPU_LOAD_TRACE
    }
}

/**
 *@}
 **/
