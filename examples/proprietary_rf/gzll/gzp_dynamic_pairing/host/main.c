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
 * $LastChangedRevision: 17930 $
 */


/**
 * This project requires a running counterpart project, which is either of the following:
 *
 * 1) nRF24Lxx device running the gzll_device_w_dynamic_pairing example from the
 * compatible version of the nRFgo SDK
 *
 * 2) nRF5x device running the gzp_device_dynamic_pairing_example example.
 *
 * The application listens for packets continuously, monitoring for pairing
 * requests, as well as normal user data.
 *
 * The Gazell pairing library uses pipe 0 and pipe 1 for encrypted communication.
 * The application will grant any request for a host ID, thus granting pairing.
 * Unencrypted packets can be received on pipe 2.
 *
 * When DATA is received, the contents of the first payload byte
 * are output on GPIO Port LEDS.
 *
 */
#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "nrf_ecb.h"
#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_gzll_error.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
#define UNENCRYPTED_DATA_PIPE     2   ///< Pipes 0 and 1 are reserved for GZP pairing and data. See nrf_gzp.h.
#define NRF_GZLLDE_RXPERIOD_DIV_2 504 ///< RXPERIOD/2 on LU1 = timeslot period on nRF5x.

#define APP_TIMER_PRESCALER       0   ///< Value of the RTC PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE   8u  ///< Size of timer operation queues.


/**
 * @brief Initialize the BSP modules.
 */
static void ui_init(void)
{
    // Initialize application timer.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 NULL);
    APP_ERROR_CHECK(err_code);

    // Set up logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Gazell dynamic pairing example. Host mode.\r\n");
    NRF_LOG_FLUSH();

    LEDS_ON(LEDS_MASK);
}


/**
 * @brief Function to control LED outputs.
 *
 * @param[in] val Desirable state of the LEDs.
 */
static void output_present(uint8_t val)
{
    uint32_t leds[] = LEDS_LIST;
    uint32_t i;

    for (i = 0; i < LEDS_NUMBER; i++)
    {
        if (val & (1 << i))
        {
            LEDS_ON(1 << (leds[i]));
        }
        else
        {
            LEDS_OFF(1 << (leds[i]));
        }
    }
}


/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main(void)
{
    // Debug helper variables
    uint32_t length;

    // Data and acknowledgement payloads
    uint8_t payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

    // Set up the user interface (buttons and LEDs)
    ui_init();

    // Initialize the Gazell Link Layer
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2); // Half RX period on an nRF24Lxx device
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Initialize the Gazell Pairing Library
    gzp_init();
    result_value = nrf_gzll_set_rx_pipes_enabled(nrf_gzll_get_rx_pipes_enabled() |
                                                 (1 << UNENCRYPTED_DATA_PIPE));
    GAZELLE_ERROR_CODE_CHECK(result_value);

    gzp_pairing_enable(true);

    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    for (;;)
    {
        gzp_host_execute();

        // If a Host ID request received
        if (gzp_id_req_received())
        {
            // Always grant a request
            gzp_id_req_grant();
        }

        length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

        if (nrf_gzll_get_rx_fifo_packet_count(UNENCRYPTED_DATA_PIPE))
        {
            if (nrf_gzll_fetch_packet_from_rx_fifo(UNENCRYPTED_DATA_PIPE, payload, &length))
            {
                output_present(payload[0]);
            }
        }
        else if (gzp_crypt_user_data_received())
        {
            if (gzp_crypt_user_data_read(payload, (uint8_t *)&length))
            {
                output_present(payload[0]);
            }
        }
    }
}


