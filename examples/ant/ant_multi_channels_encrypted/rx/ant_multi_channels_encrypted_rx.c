/*
This software is subject to the license described in the license.txt file included with
this software distribution.
You may not use this file except in compliance with this license.

Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_multi_channels_encrypted_rx.h"
#include "boards.h"
#include "app_error.h"
#include "ant_channel_config.h"
#include "ant_search_config.h"
#include "ant_encrypt_negotiation_slave.h"

// Channel configuration.
#define ANT_EXT_ASSIGN              0x00                    /**< ANT Ext Assign. */
#define ANT_CHANNEL_DEFAULT_NETWORK 0x00                    /**< ANT channel network. */
#define ANT_MSG_IDX_ID              1u                      /**< ANT message ID index. */

/** Encryption key. */
#define CRYPTO_KEY                  {0x03, 0x01, 0x04, 0x01, 0x05, 0x09, 0x02, 0x06, 0x05, 0x03, \
                                     0x05, 0x08, 0x09, 0x07, 0x09, 0x03}

/** Number of channels to open <1,ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED>. */
#define NUMBER_OF_CHANNELS_TO_OPEN  ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED


/**@brief Function to display the bottom nibble of the input byte on the board's LEDs.
 */
static void low_nibble_on_leds_display(uint8_t byte_to_display)
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


void ant_se_num_of_decrypted_channels_display(void)
{
    uint8_t  num_decrypted_channels = 0;

    for (uint32_t i = 0; i < NUMBER_OF_CHANNELS_TO_OPEN; i++)
    {
        if (ant_channel_encryp_tracking_state_get(i) == ANT_ENC_CHANNEL_STAT_TRACKING_DECRYPTED)
        {
            num_decrypted_channels++;
        }
    }
    low_nibble_on_leds_display(num_decrypted_channels);
}


void ant_se_encryp_track_handler(uint8_t ant_channel, ant_encrypt_user_evt_t event)
{
    ant_se_num_of_decrypted_channels_display();
}


void ant_se_channel_rx_broadcast_setup(void)
{
    uint32_t err_code;

    uint8_t crypto_key[] = CRYPTO_KEY;

    ANT_ENCRYPT_STACK_SETTINGS_BASE_DEF(se_conf, crypto_key, (uint8_t *)NRF_FICR->DEVICEID);

    ant_encrypt_channel_settings_t channel_crypto_settings =
    {
        .mode            = ENCRYPTION_BASIC_REQUEST_MODE,
        .key_index       = 0,
        .decimation_rate = 1
    };

    ant_channel_config_t m_channel_config =
    {
        .channel_type      = CHANNEL_TYPE_SLAVE,
        .ext_assign        = ANT_EXT_ASSIGN,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_CHANNEL_DEFAULT_NETWORK,
        .p_crypto_settings = &channel_crypto_settings
        // .device_number and .channel_number structure members are set below separately.
    };

    ant_search_config_t m_search_config =
    {
        // .device_number structure member is set below separately.
        .low_priority_timeout  = ANT_LOW_PRIORITY_TIMEOUT_DISABLE,
        .high_priority_timeout = ANT_DEFAULT_HIGH_PRIORITY_TIMEOUT,
        .search_sharing_cycles = ANT_SEARCH_SHARING_CYCLES_DISABLE,
        .search_priority       = ANT_SEARCH_PRIORITY_DEFAULT,
        .waveform              = ANT_WAVEFORM_DEFAULT,
    };


    err_code = ant_stack_encryption_config(&ANT_ENCRYPT_STACK_SETTINGS_BASE(se_conf));
    APP_ERROR_CHECK(err_code);

    ant_enc_event_handler_register(ant_se_encryp_track_handler);

    for (uint32_t i = 0; i < NUMBER_OF_CHANNELS_TO_OPEN; i++)
    {
        // fill variable channel ID.
        m_channel_config.channel_number = i;
        m_search_config.channel_number  = i;
        m_channel_config.device_number  = i + 1;

        err_code = ant_channel_init(&m_channel_config);
        APP_ERROR_CHECK(err_code);

        // Set search time-out
        err_code = ant_search_init(&m_search_config);
        APP_ERROR_CHECK(err_code);

        // Open channel
        err_code = sd_ant_channel_open(i);
        APP_ERROR_CHECK(err_code);

    }
}


void ant_se_event_handler(ant_evt_t * p_ant_event)
{
    ant_encrypt_event_handler(p_ant_event);
}


