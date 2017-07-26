/*
This software is subject to the license described in the license.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

#include "ant_search_sharing.h"
#include <stdint.h>
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "boards.h"
#include "sdk_config.h"
#include "ant_channel_config.h"
#include "ant_search_config.h"
#include "nrf_soc.h"


// Miscellaneous defines.
#define ANT_CHANNEL_DEFAULT_NETWORK    0x00         /**< ANT Channel Network. */
#define ANT_CHANNEL_NUMBER_0           0            /**< ANT Channel Number for channel 0. */
#define ANT_CHANNEL_NUMBER_1           1            /**< ANT Channel Number for channel 1. */

bool m_rx_first_ch0;                                /**< Received first message on channel 0. */
bool m_rx_first_ch1;                                /**< Received first message on channel 1. */


void ant_search_sharing_setup(void)
{
    uint32_t err_code;

    ant_channel_config_t channel_config =
    {
        .channel_type   = CHANNEL_TYPE_SLAVE,
        .ext_assign     = 0x00,
        .device_number  = 0x00,     // Wild card
        .network_number = ANT_CHANNEL_DEFAULT_NETWORK,
    };

    ant_search_config_t search_config =
    {
        .low_priority_timeout  = SEARCH_TIMEOUT,
        .high_priority_timeout = ANT_HIGH_PRIORITY_SEARCH_DISABLE,
        .search_sharing_cycles = SEARCH_CYCLES,
        .search_priority       = ANT_SEARCH_PRIORITY_DEFAULT,
        .waveform              = ANT_WAVEFORM_FAST,     // Use fast waveform to limit impact on acquisition time from searching on multiple frequencies
    };

    m_rx_first_ch0 = false;
    m_rx_first_ch1 = false;

    // Configure channel parameters for channel 0
    channel_config.channel_number    = ANT_CHANNEL_NUMBER_0;
    channel_config.device_type       = CH0_CHAN_ID_DEV_TYPE;
    channel_config.transmission_type = CH0_CHAN_ID_TRANS_TYPE;
    channel_config.channel_period    = CH0_CHAN_PERIOD;
    channel_config.rf_freq           = CH0_RF_FREQ;
    err_code                         = ant_channel_init(&channel_config);
    APP_ERROR_CHECK(err_code);

    // Configure search parameters for channel 0
    search_config.channel_number = ANT_CHANNEL_NUMBER_0;
    err_code                     = ant_search_init(&search_config);
    APP_ERROR_CHECK(err_code);

    // Configure channel parameters for channel 1
    channel_config.channel_number    = ANT_CHANNEL_NUMBER_1;
    channel_config.device_type       = CH1_CHAN_ID_DEV_TYPE;
    channel_config.transmission_type = CH1_CHAN_ID_TRANS_TYPE;
    channel_config.channel_period    = CH1_CHAN_PERIOD;
    channel_config.rf_freq           = CH1_RF_FREQ;
    err_code                         = ant_channel_init(&channel_config);
    APP_ERROR_CHECK(err_code);

    // Configure search parameters for channel 1
    search_config.channel_number = ANT_CHANNEL_NUMBER_1;
    err_code                     = ant_search_init(&search_config);
    APP_ERROR_CHECK(err_code);

    // Open both channels
    err_code = sd_ant_channel_open(ANT_CHANNEL_NUMBER_0);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ant_channel_open(ANT_CHANNEL_NUMBER_1);
    APP_ERROR_CHECK(err_code);

}


void ant_search_sharing_event_handler(ant_evt_t * p_ant_evt)
{
    switch (p_ant_evt->event)
    {
        case EVENT_RX:
            switch (p_ant_evt->channel)
            {
                case ANT_CHANNEL_NUMBER_0:
                    if (!m_rx_first_ch0)
                    {
                        m_rx_first_ch0 = true;
                        LEDS_ON(BSP_LED_0_MASK);
                    }
                    break;

                case ANT_CHANNEL_NUMBER_1:
                    if (!m_rx_first_ch1)
                    {
                        m_rx_first_ch1 = true;
                        LEDS_ON(BSP_LED_1_MASK);
                    }
                    break;
            }
            break;

        default:
            break; // No implementation needed
    }
}


