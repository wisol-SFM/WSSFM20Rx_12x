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
 * $LastChangedRevision: 25419 $
 */

/**
 * This project can be used as a starting point for developing a nRF5x series
 * mouse or keyboard using Gazell for communicating with a legacy nRF24LU1
 * USB dongle. It can communicate "out of the box" with the
 * Dongle reference design and the nRFreadySimplePairing.exe application,
 * that can be found in the nRFready Desktop v1.2.3.
 *
 * This project sends mouse movement packets to the USB dongle when pin 1
 * is low (button connected to pin 1 pressed) and sends the 'a' keyboard
 * character to the USB dongle when pin 2 goes from high to low (button
 * connected to pin 2 pressed).
 */

#include <stdio.h>
#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_gzllde_params.h"
#include "mouse_sensor_emulator.h"
#include "keyboard_emulator.h"
#include "nrf_gzll_error.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*****************************************************************************/
/** @name Configuration                                                      */
/*****************************************************************************/
#define SEND_KEYBOARD_DATA_BUTTON_ID 0 ///< GPIO pin for reading from a button to emulate a keypress.
#define SEND_MOUSE_DATA_BUTTON_ID    1 ///< GPIO pin for reading from a button to emulate a mouse movement.


/*****************************************************************************/
/** @name Static (internal) functions. */
/*****************************************************************************/

/**
 * Checks to see whether the mouse sensor has data to send. If so, adds this
 * to the (unencrypted) Gazell TX FIFO on pipe NRFR_MOUSE_EP.
 */
static void read_mouse_and_send(void);


/**
 * Checks to see whether the mouse sensor has data and send unencrypted.
 *
 * If so, adds this to the (unencrypted) Gazell TX FIFO on pipe NRFR_MOUSE_EP.
 */

/**
 * Checks to see whether the keyboard has data and send encrypted.
 *
 * If the Device does net yet have the system address it will try to
 * obtain it. After obtaining the system address it will attempt to obtain
 * the Host ID.
 * The keyboard data is discarded if pairing is not successful.
 * It may take a few attempts to obtain the Host ID as this may not be
 * yet generated at the Host.
 */
static void read_keyboard_and_send(void);

/**
 * Send a Host ID request and process the response.
 *
 * If the request was rejected or failed (i.e. timed out), system_addr_received
 * will be reset and the pairing process will begin on the next keypress.
 *
 * If the request was received, subsequent keyboard data will be transmitted
 * on an encrypted link.
 *
 * If teh request is still pending, nothing is done. Further keypresses
 * will initiate another
 *
 * @return The result of the Host ID request.
 */
static gzp_id_req_res_t send_host_id_req(void);
/** @} */

static void error_report(void);

static bool host_id_received     = false;     ///< Host ID received.
static bool system_addr_received = false;     ///< System address receivedfrom Host.
static bool dyn_key_ok           = false;     ///< Dynamic key is up to date.

static bool prev_host_id_received     = true; ///< Host ID received.
static bool prev_system_addr_received = true; ///< System address receivedfrom Host.
static bool prev_dyn_key_ok           = true; ///< Dynamic key is up to date.


/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/


int main()
{
    bool     result_value = false;
    uint32_t err_code;

// lint -save -e514 Unusual use of a boolean expression (use of &= assignment).

    UNUSED_VARIABLE(bsp_init(BSP_INIT_BUTTONS, 0, NULL));

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Desktop emulator example\r\n");
    NRF_LOG_FLUSH();

    // Initialize and enable "mouse sensor"
    result_value = mouse_sensor_init(MOUSE_SENSOR_SAMPLE_PERIOD_8_MS);
    APP_ERROR_CHECK_BOOL(result_value);

    mouse_sensor_enable();

    // Initialize and enable Gazell
    result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Ensure Gazell parameters are configured.
    result_value = nrf_gzll_set_max_tx_attempts(150);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_device_channel_selection_policy(
        NRF_GZLLDE_DEVICE_CHANNEL_SELECTION_POLICY);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_sync_lifetime(0); // Asynchronous mode, more efficient for pairing.
    GAZELLE_ERROR_CODE_CHECK(result_value);

    switch (gzp_get_pairing_status())
    {
        case -2:
            host_id_received     = false;
            system_addr_received = false;
            break;

        case -1:
            host_id_received     = false;
            system_addr_received = true;
            break;

        default:
            host_id_received     = true;
            system_addr_received = true;
    }

    gzp_init();

    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    while (1)
    {
        // If BUTTON_SEND_MOUSE_DATA button is pressed.
        bool send_mouse_data_button_pressed;
        err_code = bsp_button_is_pressed(SEND_MOUSE_DATA_BUTTON_ID,
                                         &(send_mouse_data_button_pressed));
        if ( send_mouse_data_button_pressed)
        {
            read_mouse_and_send();
        }

        // If BUTTON_SEND_KEYBOARD_DATA button is pressed
        bool send_keyboard_data_button_pressed;
        err_code = bsp_button_is_pressed(SEND_KEYBOARD_DATA_BUTTON_ID,
                                         &(send_keyboard_data_button_pressed));
        if (send_keyboard_data_button_pressed)
        {
            read_keyboard_and_send();
        }

        error_report();

    }
// lint -restore
}


void mouse_sensor_new_sample_generated_cb()
{
    /*
       This callback is called every time the mouse sensor
       generates a new sample. We could select to add mouse packets to the
       TX FIFO here.
     */
}


static void read_mouse_and_send(void)
{
// lint -save -e514 Unusual use of a boolean expression (use of &= assignment).
    bool mouse_send_ok;

    // If the "mouse sensor" has data ready for read-out.
    if (mouse_sensor_data_is_ready())
    {
        uint8_t mouse_packet[NRFR_MOUSE_MOV_PACKET_LENGTH];

        // Get packet from "mouse sensor".
        if (mouse_sensor_read(mouse_packet))
        {
            // Wait in case the FIFOs are full.
            while (!nrf_gzll_ok_to_add_packet_to_tx_fifo(NRFR_MOUSE_EP))
                ;

            // Add mouse packet to the mouse pipe's TX FIFO.
            mouse_send_ok = nrf_gzll_add_packet_to_tx_fifo(NRFR_MOUSE_EP,
                                                           mouse_packet,
                                                           NRFR_MOUSE_MOV_PACKET_LENGTH);
            if (mouse_send_ok)
            {
                NRF_LOG_INFO("Mouse sent OK\r\n");
            }
            else
            {
                NRF_LOG_INFO("Mouse sent FAILED\r\n");
            }

            NRF_LOG_FLUSH();
        }
    }


// lint -restore
}


static void read_keyboard_and_send(void)
{
    uint8_t keyboard_packet[NRFR_KEYBOARD_PACKET_LENGTH];

    // "Scan" keyboard.
    keyboard_get_non_empty_packet(keyboard_packet);

    // Send address request if required.
    if (!host_id_received && !system_addr_received)
    {
        system_addr_received = gzp_address_req_send();
    }

    /* Send Host ID request if required. This may take a few attempts
     * as the Host may require some time to generate the Host ID. */
    if (!host_id_received && system_addr_received )
    {
        while (send_host_id_req() == GZP_ID_RESP_PENDING)
            ;
    }

    /* After receiving the Host ID we send one packet in order
     * to update the dynamic key.
     */
    if (host_id_received && !dyn_key_ok)
    {
        bool keyboard_send_ok = true;
        keyboard_send_ok = gzp_crypt_data_send(keyboard_packet,
                                               NRFR_KEYBOARD_PACKET_LENGTH);

        if (!keyboard_send_ok)
        {
            host_id_received = false;
        }
        else
        {
            dyn_key_ok = true;
        }
    }

    /* If we have the Host ID and dynamic key we can transmit encrypted data.
     */
    if (host_id_received && dyn_key_ok)
    {
        bool keyboard_send_ok = true;
        keyboard_send_ok = gzp_crypt_data_send(keyboard_packet,
                                               NRFR_KEYBOARD_PACKET_LENGTH);

        if (keyboard_send_ok)
        {
            NRF_LOG_INFO("Keyboard sent OK\r\n");
        }
        else
        {
            NRF_LOG_INFO("Keyboard sent FAILED\r\n");
        }

        if (keyboard_send_ok)
        {
            // Wait until button is released.
            bool button_pressed;

            do
            {
                UNUSED_VARIABLE(bsp_button_is_pressed(SEND_KEYBOARD_DATA_BUTTON_ID,
                                                      &(button_pressed)));
            }
            while (button_pressed);

            // Send empty keyboard packet to release all keys.
            keyboard_get_empty_packet(keyboard_packet);
            keyboard_send_ok = gzp_crypt_data_send(keyboard_packet, NRFR_KEYBOARD_PACKET_LENGTH);
        }

        if (!keyboard_send_ok)
        {
            dyn_key_ok = false;
        }

        NRF_LOG_FLUSH();
    }

}


static gzp_id_req_res_t send_host_id_req(void)
{
    gzp_id_req_res_t id_resp;

    // Try sending "Host ID" request
    id_resp = gzp_id_req_send();

    switch (id_resp)
    {
        case GZP_ID_RESP_REJECTED:
        case GZP_ID_RESP_FAILED:
            host_id_received     = false;
            system_addr_received = false;
            break;

        case GZP_ID_RESP_GRANTED:
            host_id_received     = true;
            system_addr_received = true;
            break;

        case GZP_ID_RESP_PENDING:
        default:
            break;
    }

    return id_resp;
}


static void error_report(void)
{
    if (~system_addr_received & prev_system_addr_received)
    {
        prev_system_addr_received = false;
        NRF_LOG_INFO("System address not received\r\n");
    }
    else if (system_addr_received)
    {
        prev_system_addr_received = true;
    }

    if (~host_id_received & prev_host_id_received)
    {
        prev_host_id_received = false;
        NRF_LOG_INFO("Host ID not received\r\n");
    }
    else if (host_id_received)
    {
        prev_host_id_received = true;
    }

    if (~dyn_key_ok & prev_dyn_key_ok)
    {
        prev_dyn_key_ok = false;
        NRF_LOG_INFO("Dyn Key not received\r\n");
    }
    else if (dyn_key_ok)
    {
        prev_dyn_key_ok = true;
    }

    NRF_LOG_FLUSH();
}


