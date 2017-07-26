/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.

Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

#include <stdint.h>
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "boards.h"
#include "sdk_config.h"
#include "ant_channel_config.h"
#include "ant_multi_channels_tx.h"


#define ANT_EXT_ASSIGN                  0x00                            /**< ANT Ext Assign. */
#define ANT_CHANNEL_DEFAULT_NETWORK     0x00                            /**< ANT Channel Network. */

// Private variables
static uint8_t m_counter[ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED]   = {0};  /**< Counters to increment the ANT broadcast data payload. */
static uint8_t m_broadcast_data[ANT_STANDARD_DATA_PAYLOAD_SIZE] = {0};  /**< Primary data transmit buffers. */
static uint8_t m_num_open_channels                              = 0;    /**< Number of channels open */

/**@brief Function to display the bottom nibble of the input byte
 */
static void ant_scaleable_display_byte_on_leds(uint8_t byte_to_display)
{
    LEDS_OFF(LEDS_MASK);

    uint32_t mask = 0;
    if ((byte_to_display & 0x01) == 1)
    {
        mask = mask | BSP_LED_3_MASK;
    }
    if (((byte_to_display >> 1) & 0x01) == 1)
    {
        mask = mask | BSP_LED_2_MASK;
    }
    if (((byte_to_display >> 2) & 0x01) == 1)
    {
        mask = mask | BSP_LED_1_MASK;
    }
    if (((byte_to_display >> 3) & 0x01) == 1)
    {
        mask = mask | BSP_LED_0_MASK;
    }

    LEDS_ON(mask);
}


void ant_scaleable_channel_tx_broadcast_setup(void)
{
    uint32_t err_code;
    uint32_t i = 0;

    ant_channel_config_t channel_config =
    {
        .channel_number     = i,
        .channel_type       = CHANNEL_TYPE_MASTER,
        .ext_assign         = ANT_EXT_ASSIGN,
        .rf_freq            = RF_FREQ,
        .transmission_type  = CHAN_ID_TRANS_TYPE,
        .device_type        = CHAN_ID_DEV_TYPE,
        .device_number      = i + 1,
        .channel_period     = CHAN_PERIOD,
        .network_number     = ANT_CHANNEL_DEFAULT_NETWORK,
    };

    for (i = 0; i < ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED; i++)
    {
        // Configure channel
        channel_config.channel_number = i;
        channel_config.device_number  = i + 1;

        err_code = ant_channel_init(&channel_config);
        APP_ERROR_CHECK(err_code);

        // Open channel
        err_code = sd_ant_channel_open(i);
        APP_ERROR_CHECK(err_code);

        m_num_open_channels++;
        ant_scaleable_display_byte_on_leds(m_num_open_channels);
    }
}


void ant_scaleable_event_handler(ant_evt_t * p_ant_evt)
{
    uint32_t err_code;

    switch (p_ant_evt->event)
    {
        // ANT broadcast success.
        // Increment the counter and send a new broadcast.
        case EVENT_TX:
            // Increment the counter.
            m_counter[p_ant_evt->channel]++;

            // Assign a new value to the broadcast data.
            m_broadcast_data[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = m_counter[p_ant_evt->channel];

            // Broadcast the data.
            err_code = sd_ant_broadcast_message_tx(p_ant_evt->channel,
                                                   ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                   m_broadcast_data);
            APP_ERROR_CHECK(err_code);

            break;

        default:
            break;
    }
}
