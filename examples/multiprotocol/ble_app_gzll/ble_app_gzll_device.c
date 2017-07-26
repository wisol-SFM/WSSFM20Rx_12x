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

/** @file
 *
 * @{
 * @ingroup ble_app_gzll_gazell_part
 */

#include "ble_app_gzll_device.h"
#include <stdint.h>
#include "nordic_common.h"
#include "app_error.h"
#include "nrf_gzll.h"
#include "nrf_gzll_error.h"
#include "ble_app_gzll_ui.h"
#include "bsp.h"

#define MAX_TX_ATTEMPTS         100  /**< Maximum number of Transmit attempts.*/
#define DUMMY_PACKET            0x80 /**< First payload.*/
#define NB_TX_WITH_SAME_PAYLOAD 3    /**< Number of times each packet is sent (this avoid changing the payload too quickly and allows to have a visible LED pattern on the receiver side).*/
#define PAYLOAD_SIZE            8    /**< Size of the payload to send over Gazell.*/
#define PIPE_TO_HOST            0    /**< Pipe number. */


static uint8_t m_gzll_packet[PAYLOAD_SIZE];


/**@brief Function for starting Gazell functionality.
 */
void gzll_app_start(void)
{
    GAZELLE_ERROR_CODE_CHECK(nrf_gzll_init(NRF_GZLL_MODE_DEVICE));
    GAZELLE_ERROR_CODE_CHECK(nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS));
    GAZELLE_ERROR_CODE_CHECK(nrf_gzll_enable());

    // Add a packet to the TX FIFO to start the data transfer.
    // Next packets to send will be added.
    m_gzll_packet[0] = DUMMY_PACKET;
    GAZELLE_ERROR_CODE_CHECK(nrf_gzll_add_packet_to_tx_fifo(PIPE_TO_HOST,
                                                            m_gzll_packet,
                                                            PAYLOAD_SIZE));
}


void gzll_app_stop()
{
    // Disable gazell.
    nrf_gzll_disable();

    // Wait for Gazell to shut down.
    while (nrf_gzll_is_enabled())
    {

    }

    // Clean up after Gazell.
    NVIC_DisableIRQ(RADIO_IRQn);
    NVIC_DisableIRQ(TIMER2_IRQn);
    NVIC_DisableIRQ(SWI0_IRQn);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_ClearPendingIRQ(TIMER2_IRQn);
    NVIC_ClearPendingIRQ(SWI0_IRQn);

}


/**@brief Callback function for Gazell Transmit Success. Adds new packet to tx fifo.
 */
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    static int cpt = 0;
    uint8_t    dummy[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
    uint32_t   dummy_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    uint32_t   err_code;

    // If an ACK was received, we send another packet.
    err_code = bsp_indication_set(BSP_INDICATE_SENT_OK);
    APP_ERROR_CHECK(err_code);

    if (tx_info.payload_received_in_ack)
    {
        // if ack was sent with payload, pop them from rx fifo.
        GAZELLE_ERROR_CODE_CHECK(nrf_gzll_fetch_packet_from_rx_fifo(pipe, dummy, &dummy_length));
    }

    cpt++;

    // Update transmitted data.
    if (cpt > NB_TX_WITH_SAME_PAYLOAD)
    {
        cpt = 0;

        m_gzll_packet[0] = ~(m_gzll_packet[0]);
        if (m_gzll_packet[0] == DUMMY_PACKET)
        {
            m_gzll_packet[0] = 0x00;
        }

        m_gzll_packet[0] <<= 1;
        if (m_gzll_packet[0] == 0)
        {
            m_gzll_packet[0]++;
        }

        m_gzll_packet[0] = ~(m_gzll_packet[0]);
    }

    //  Add next packet to tx fifo.
    GAZELLE_ERROR_CODE_CHECK(nrf_gzll_add_packet_to_tx_fifo(pipe, m_gzll_packet, PAYLOAD_SIZE));

}


/**@brief Callback function for Gazell Transmit fail. Resends the current packet.
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    uint8_t  dummy[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
    uint32_t dummy_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    uint32_t err_code;

    // If the transmission failed, send a new packet.
    err_code = bsp_indication_set(BSP_INDICATE_SEND_ERROR);
    APP_ERROR_CHECK(err_code);

    if (tx_info.payload_received_in_ack)
    {
        // if ack was sent with payload, pop them from rx fifo.
        GAZELLE_ERROR_CODE_CHECK(nrf_gzll_fetch_packet_from_rx_fifo(pipe, dummy, &dummy_length));
    }

    GAZELLE_ERROR_CODE_CHECK(nrf_gzll_add_packet_to_tx_fifo(pipe, m_gzll_packet, PAYLOAD_SIZE));
}


/**@brief Callback function for Gazell Receive Data Ready. Flushes the receive's FIFO.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    // We dont expect to receive any data in return, but if it happens we flush the RX fifo.
    GAZELLE_ERROR_CODE_CHECK(nrf_gzll_flush_rx_fifo(pipe));
}


/**@brief Callback function for Gazell Disabled - Not needed in this example.
 */
void nrf_gzll_disabled()
{
}


/**
 * @}
 */
