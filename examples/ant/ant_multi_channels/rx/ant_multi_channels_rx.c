/*
This software is subject to the license described in the license.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

#include <stdint.h>
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "boards.h"
#include "sdk_config.h"
#include "ant_channel_config.h"
#include "ant_search_config.h"
#include "ant_multi_channels_rx.h"


#define ANT_EXT_ASSIGN                 0x00                               /**< ANT Ext Assign. */
#define ANT_CHANNEL_DEFAULT_NETWORK    0x00                               /**< ANT Channel Network. */

// Private variables
static bool m_tracking_channels[ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED] = { false };     /**< Array to keep track of which channels are currently tracking */



/**@brief Function to display the bottom nibble of the input byte on the board's LEDs
 *
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


/**@brief Function for handling ANT GO_TO_SEARCH events
 *
 */
static void goto_search_handler(uint8_t channel)
{
    m_tracking_channels[channel] = false;
    ant_scaleable_display_num_tracking_channels();
}


/**@brief Function for handling ANT RX events
 *
 */
static void message_received_handler(uint8_t ant_channel, uint8_t const * const p_event_message_buffer)
{
    ANT_MESSAGE * p_ant_message = (ANT_MESSAGE*)p_event_message_buffer;

    switch (p_ant_message->ANT_MESSAGE_ucMesgID)
    {
        // Broadcast data received
        case MESG_BROADCAST_DATA_ID:
            m_tracking_channels[ant_channel] = true;
            ant_scaleable_display_num_tracking_channels();
            break;

        default:
            break;
    }
}


void ant_scaleable_display_num_tracking_channels()
{
    uint8_t  num_tracking_channels = 0;
    uint32_t i;

    for (i = 0; i < ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED; i++)
    {
        if (m_tracking_channels[i])
        {
            num_tracking_channels++;
        }
    }
    ant_scaleable_display_byte_on_leds(num_tracking_channels);
}


void ant_scaleable_channel_rx_broadcast_setup(void)
{
    uint32_t err_code;
    uint32_t i = 0;

    ant_channel_config_t channel_config =
    {
        .channel_number     = i,
        .channel_type       = CHANNEL_TYPE_SLAVE,
        .ext_assign         = ANT_EXT_ASSIGN,
        .rf_freq            = RF_FREQ,
        .transmission_type  = CHAN_ID_TRANS_TYPE,
        .device_type        = CHAN_ID_DEV_TYPE,
        .device_number      = i + 1,
        .channel_period     = CHAN_PERIOD,
        .network_number     = ANT_CHANNEL_DEFAULT_NETWORK,
    };

    ant_search_config_t ant_search_config   = DEFAULT_ANT_SEARCH_CONFIG(i);
    ant_search_config.high_priority_timeout = ANT_HIGH_PRIORITY_TIMEOUT_DISABLE;

    for (i = 0; i < ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED; i++)
    {
        // Configure channel
        channel_config.channel_number = i;
        channel_config.device_number  = i + 1;

        err_code = ant_channel_init(&channel_config);
        APP_ERROR_CHECK(err_code);

        // Set search timeout
        ant_search_config.channel_number = i;
        err_code = ant_search_init(&ant_search_config);
        APP_ERROR_CHECK(err_code);

        // Open channel
        err_code = sd_ant_channel_open(i);
        APP_ERROR_CHECK(err_code);
    }
}


void ant_scaleable_event_handler(ant_evt_t * p_ant_evt)
{
    switch (p_ant_evt->event)
    {
        case EVENT_RX_FAIL_GO_TO_SEARCH:
            goto_search_handler(p_ant_evt->channel);
            break;

        case EVENT_RX:
            message_received_handler(p_ant_evt->channel, p_ant_evt->msg.evt_buffer);
            break;

        default:
            break;
    }
}
