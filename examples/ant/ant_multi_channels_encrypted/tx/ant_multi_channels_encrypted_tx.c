/*
This software is subject to the license described in the license.txt file included with
this software distribution.
You may not use this file except in compliance with this license.

Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

#include <stdint.h>
#include <stdlib.h>
#include "app_error.h"
#include "boards.h"
#include "nrf_sdm.h"
#include "ant_interface.h"
#include "nordic_common.h"
#include "ant_parameters.h"
#include "ant_multi_channels_encrypted_tx.h"
#include "ant_channel_config.h"
#include "ant_encrypt_config.h"
#include "sdk_config.h"

#define ANT_EXT_ASSIGN     0x00                    /**< ANT Ext Assign. */

/** ANT channel network. */
#define ANT_CHANNEL_DEFAULT_NETWORK 0x00

/** Size of the broadcast data buffer. */
#define BROADCAST_DATA_BUFFER_SIZE  ANT_STANDARD_DATA_PAYLOAD_SIZE

/** Encryption key. */
#define CRYPTO_KEY                 {0x03, 0x01, 0x04, 0x01, 0x05, 0x09, 0x02, 0x06, 0x05, 0x03, \
                                    0x05, 0x08, 0x09, 0x07, 0x09, 0x03}

/** Number of channels to open <1,ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED>. */
#define NUMBER_OF_CHANNELS_TO_OPEN ANT_CONFIG_TOTAL_CHANNELS_ALLOCATED

// Private variables.
/** Counters to increment the ANT broadcast data payload. */
static uint8_t m_counter[NUMBER_OF_CHANNELS_TO_OPEN] = { 0u };

/** Primary data transmit buffers. */
static uint8_t m_broadcast_data[BROADCAST_DATA_BUFFER_SIZE] = { 0 };

/** Number of channels open. */
static uint8_t m_num_open_channels = 0;

/**@brief Function to display the bottom nibble of the input byte.
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


void ant_multi_channels_encrypted_channel_tx_broadcast_setup(void)
{
    uint32_t err_code;
    uint32_t i;

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
        .channel_type      = CHANNEL_TYPE_MASTER,
        .ext_assign        = ANT_EXT_ASSIGN,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_CHANNEL_DEFAULT_NETWORK,
        .p_crypto_settings = &channel_crypto_settings
        // .device_number and .channel_number structure members are set below separately.
    };

    err_code = ant_stack_encryption_config(&ANT_ENCRYPT_STACK_SETTINGS_BASE(se_conf));
    APP_ERROR_CHECK(err_code);

    for (i = 0; i < NUMBER_OF_CHANNELS_TO_OPEN; i++)
    {
        // Fill variable channel ID.
        m_channel_config.channel_number = i;
        m_channel_config.device_number  = i + 1;

        err_code = ant_channel_init(&m_channel_config);
        APP_ERROR_CHECK(err_code);

        // Open channel.
        err_code = sd_ant_channel_open(i);
        APP_ERROR_CHECK(err_code);

        m_num_open_channels++;
    }

    low_nibble_on_leds_display(m_num_open_channels);
}


void ant_multi_channels_encrypted_event_handler(ant_evt_t * p_ant_evt)
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
            m_broadcast_data[BROADCAST_DATA_BUFFER_SIZE - 1] = m_counter[p_ant_evt->channel];

            // Broadcast the data.
            err_code = sd_ant_broadcast_message_tx(p_ant_evt->channel,
                                                   BROADCAST_DATA_BUFFER_SIZE,
                                                   m_broadcast_data);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


