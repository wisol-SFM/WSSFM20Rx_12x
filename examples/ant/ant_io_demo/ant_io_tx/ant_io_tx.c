/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

#include "ant_io_tx.h"
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

//ANT Channels
#define ANT_CHANNEL_NUMBER              0x00                    /**< ANT Channel 0. */
#define EXT_ASSIGN                      0x00                    /**< ANT Ext Assign. */
#define ANT_CHANNEL_DEFAULT_NETWORK     0x00                    /**< ANT Network (default public network). */

// Data Page Numbers
#define DIGITALIO_DATA_PID              1u                      /**< Page number: digital data. */

// Static variables and buffers.
static uint8_t m_broadcast_data[ANT_STANDARD_DATA_PAYLOAD_SIZE];    /**< Primary data transmit buffer. */
static uint8_t m_rx_input_pin_state = 0xFF;                         /**< State of received digital data, from the other node. */
static uint8_t m_tx_input_pin_state = 0xFF;                         /**< State of digital inputs in this node, for transmission. */


/**@brief Encode current state of buttons
 *
 * Configure bitfield encoding the state of the buttons
 * Bit 0 = 0 if BSP_BUTTON_0 is pressed
 * Bit 1 = 0 if BSP_BUTTON_1 is pressed
 * Bit 2 = 0 if BSP_BUTTON_2 is pressed
 * Bit 3 = 0 if BSP_BUTTON_3 is pressed
 * Bit 4-7 = 1 (unused)
 */
static void button_state_encode(void)
{
    uint32_t err_code;
    bool     button0;
    bool     button1;
    bool     button2;
    bool     button3;

    err_code = bsp_button_is_pressed(0, &button0);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_button_is_pressed(1, &button1);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_button_is_pressed(2, &button2);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_button_is_pressed(3, &button3);
    APP_ERROR_CHECK(err_code);

    m_tx_input_pin_state = 0xFF;

    if (button0)
    {
        m_tx_input_pin_state &= 0xFE;
    }
    if (button1)
    {
        m_tx_input_pin_state &= 0xFD;
    }
    if (button2)
    {
        m_tx_input_pin_state &= 0xFB;
    }
    if (button3)
    {
        m_tx_input_pin_state &= 0xF7;
    }
}


/**@brief Formats page with current button state and sends data
 * Byte 0   = Page number (Digital I/O Data)
 * Byte 1-6 = Reserved
 * Byte 7   = State of digital inputs
 */
static void handle_transmit()
{
    uint32_t err_code;

    button_state_encode();

    m_broadcast_data[0] = DIGITALIO_DATA_PID;
    m_broadcast_data[1] = 0xFF;
    m_broadcast_data[2] = 0xFF;
    m_broadcast_data[3] = 0xFF;
    m_broadcast_data[4] = 0xFF;
    m_broadcast_data[5] = 0xFF;
    m_broadcast_data[6] = 0xFF;
    m_broadcast_data[7] = m_tx_input_pin_state;

    err_code = sd_ant_broadcast_message_tx(ANT_CHANNEL_NUMBER,
                                           ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                           m_broadcast_data);
    APP_ERROR_CHECK(err_code);
}


/**@brief Turns on LEDs according to the contents of the received data page.
 */
static void led_state_set()
{
    uint8_t led_state_field = ~m_rx_input_pin_state;

    if (led_state_field & 1)
        LEDS_ON(BSP_LED_0_MASK);
    else
        LEDS_OFF(BSP_LED_0_MASK);

    if (led_state_field & 2)
        LEDS_ON(BSP_LED_1_MASK);
    else
        LEDS_OFF(BSP_LED_1_MASK);

    if (led_state_field & 4)
        LEDS_ON(BSP_LED_2_MASK);
    else
        LEDS_OFF(BSP_LED_2_MASK);

    if (led_state_field & 8)
        LEDS_ON(BSP_LED_3_MASK);
    else
        LEDS_OFF(BSP_LED_3_MASK);
}


void ant_io_tx_setup(void)
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
        .device_number     = NRF_FICR->DEVICEID[0],
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_CHANNEL_DEFAULT_NETWORK,
    };

    // Configure channel parameters
    err_code = ant_channel_init(&channel_config);
    APP_ERROR_CHECK(err_code);

    // Open channel.
    err_code = sd_ant_channel_open(ANT_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}


void ant_io_tx_event_handler(ant_evt_t * p_ant_evt)
{
    ANT_MESSAGE * p_message = (ANT_MESSAGE *) p_ant_evt->msg.evt_buffer;

    switch (p_ant_evt->event)
    {
        case EVENT_RX:
            if (p_message->ANT_MESSAGE_aucPayload[0] == DIGITALIO_DATA_PID)
            {
                // Set LEDs according to Received Digital IO Data Page
                m_rx_input_pin_state = p_message->ANT_MESSAGE_aucPayload[7];
                led_state_set();
            }
            break;

        case EVENT_TX:
            // Transmit data on the reverse direction every channel period
            handle_transmit();
            break;

        default:
            break;
    }
}



