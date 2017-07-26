/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/

#include "ant_message_types_master.h"
#include <stdint.h>
#include "string.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "ant_error.h"
#include "boards.h"
#include "sdk_config.h"
#include "ant_channel_config.h"
#include "nrf_soc.h"
#include "nrf_delay.h"

// I/O configuration
#define LED_BROADCAST                   BSP_LED_0_MASK
#define LED_ACKNOWLEDGED                BSP_LED_1_MASK
#define LED_BURST                       BSP_LED_2_MASK

// Channel configuration.
#define ANT_CHANNEL_NUMBER              0x00                 /**< ANT Channel 0. */
#define EXT_ASSIGN                      0x00                 /**< ANT Ext Assign. */
#define ANT_CUSTOM_TRANSMIT_POWER       0u                   /**< ANT Custom Transmit Power (Invalid/Not Used). */
#define ANT_CHANNEL_DEFAULT_NETWORK     0x00                 /**< ANT Channel Network. */
#define BROADCAST_DATA_BUFFER_SIZE      8u                   /**< Size of the broadcast data buffer. */
#define BURST_BLOCK_SIZE                32u                  /**< Size of data block transmitted via burst. Size must be divisible by 8. */

// Static variables and buffers.
static uint8_t m_tx_buffer[BROADCAST_DATA_BUFFER_SIZE]; /**< Primary data (Broadcast/Acknowledged) transmit buffer. */
static uint8_t m_counter = 1u;                               /**< Counter to increment the ANT broadcast data payload. */
static uint8_t m_burst_data[BURST_BLOCK_SIZE];               /**< Burst data transmit buffer. */

// State Machine
enum MESSAGE_TYPES_MASTER_STATES
{
    BROADCAST,
    ACKNOWLEDGED,
    BURST
} state_message_types;



void ant_message_types_master_setup(void)
{
    uint32_t err_code;

    ant_channel_config_t channel_config =
    {
        .channel_number    = ANT_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_MASTER,
        .ext_assign        = EXT_ASSIGN,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .device_number     = (uint16_t) (NRF_FICR->DEVICEID[0]),
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_CHANNEL_DEFAULT_NETWORK,
    };

    err_code = ant_channel_init(&channel_config);
    APP_ERROR_CHECK(err_code);

    //Set Tx Power
    err_code = sd_ant_channel_radio_tx_power_set(ANT_CHANNEL_NUMBER,
                                                 RADIO_TX_POWER_LVL_3,
                                                 ANT_CUSTOM_TRANSMIT_POWER);
    APP_ERROR_CHECK(err_code);

    // Open channel.
    err_code = sd_ant_channel_open(ANT_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);

    // Write counter value to last byte of the broadcast data.
    // The last byte is chosen to get the data more visible in the end of an printout
    // on the recieving end.
    memset(m_tx_buffer, 0, BROADCAST_DATA_BUFFER_SIZE);
    m_tx_buffer[BROADCAST_DATA_BUFFER_SIZE - 1] = m_counter;

    // Configure the initial payload of the broadcast data
    err_code = sd_ant_broadcast_message_tx(ANT_CHANNEL_NUMBER,
                                           BROADCAST_DATA_BUFFER_SIZE,
                                           m_tx_buffer);
    APP_ERROR_CHECK(err_code);

    //Set state to broadcasting
    state_message_types = BROADCAST;
}


void ant_message_types_master_bsp_evt_handler(bsp_event_t evt)
{
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            state_message_types = BROADCAST;
            break;

        case BSP_EVENT_KEY_1:
            state_message_types = ACKNOWLEDGED;
            break;

        case BSP_EVENT_KEY_2:
            state_message_types = BURST;
            break;

        default:
            break; // no implementation needed
    }
}


void ant_message_types_master_event_handler(ant_evt_t * p_ant_evt)
{
    uint32_t err_code;
    uint32_t led_output = LED_BROADCAST;

    switch (p_ant_evt->event)
    {
        // ANT broadcast/Acknowledged/Burst Success
        // Send the next message according to the current state and increment the counter.
        case EVENT_TX:                      // Intentional fall through
        case EVENT_TRANSFER_TX_COMPLETED:   // Intentional fall through
        case EVENT_TRANSFER_TX_FAILED:
            LEDS_OFF(LEDS_MASK);
            m_tx_buffer[BROADCAST_DATA_BUFFER_SIZE - 1] = m_counter;

            if (state_message_types == BROADCAST)
            {
                // Send as broadcast
                err_code = sd_ant_broadcast_message_tx(ANT_CHANNEL_NUMBER,
                                                       BROADCAST_DATA_BUFFER_SIZE,
                                                       m_tx_buffer);
                APP_ERROR_CHECK(err_code);

                led_output = LED_BROADCAST;
            }
            else if (state_message_types == ACKNOWLEDGED)
            {
                // Send as acknowledged
                err_code = sd_ant_acknowledge_message_tx(ANT_CHANNEL_NUMBER,
                                                         BROADCAST_DATA_BUFFER_SIZE,
                                                         m_tx_buffer);
                APP_ERROR_CHECK(err_code);

                led_output = LED_ACKNOWLEDGED;
            }
            else if (state_message_types == BURST)
            {
                // If this is a new message, populate the burst buffer
                // with new dummy data.  Otherwise, will retry sending the
                // same content.
                if (p_ant_evt->event != EVENT_TRANSFER_TX_FAILED)
                {
                    for (uint32_t i = 0; i < BURST_BLOCK_SIZE; i++)
                    {
                        m_burst_data[i] = m_counter;
                        m_counter++;
                    }
                }

                // Queue a Burst Transfer.  Since this is a small burst, queue entire burst.
                err_code = sd_ant_burst_handler_request(ANT_CHANNEL_NUMBER,
                                                        BURST_BLOCK_SIZE,
                                                        m_burst_data,
                                                        (BURST_SEGMENT_START | BURST_SEGMENT_END));
                APP_ERROR_CHECK(err_code);

                led_output = LED_BURST;
            }
            // Activate LED for 20ms
            LEDS_ON(led_output);
            nrf_delay_ms(20);
            LEDS_OFF(led_output);
            m_counter++;
            break;

        case TRANSFER_IN_PROGRESS:              //Intentional fall through
        case TRANSFER_SEQUENCE_NUMBER_ERROR:    //Intentional fall through
        case TRANSFER_IN_ERROR:                 //Intentional fall through
        case TRANSFER_BUSY:
            // Ignore these events; will retry burst transfer when we get the EVENT_TRANSFER_TX_FAILED event.
            break;

        default:
            break; // No implementation needed

    }
}

