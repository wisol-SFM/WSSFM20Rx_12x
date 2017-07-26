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
 * $LastChangedRevision: 40042 $
 */

/**
 * This project requires that a host that runs the
 * @ref gzll_host_m_ack_payload_example example is used as a counterpart for
 * receiving the data. This can be on either an nRF5x device or an nRF24Lxx device
 * running the \b gzll_host_m_ack_payload example in the nRFgo SDK.
 *
 * This example sends a packet and adds a new packet to the TX queue every time
 * it receives an ACK. Before adding a packet to the TX queue, the contents of
 * the GPIO Port BUTTONS is copied to the first payload byte (byte 0).
 * When an ACK is received, the contents of the first payload byte of
 * the ACK are output on GPIO Port LEDS.
 */

#include "nrf_gzll.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_gzll_error.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
#define PIPE_NUMBER             0   /**< Pipe 0 is used in this example. */

#define TX_PAYLOAD_LENGTH       1   /**< 1-byte payload length is used when transmitting. */
#define MAX_TX_ATTEMPTS         100 /**< Maximum number of transmission attempts */

#define APP_TIMER_PRESCALER     0   /**< Value of the RTC PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 8u  /**< Size of timer operation queues. */


static uint8_t m_data_payload[TX_PAYLOAD_LENGTH];                /**< Payload to send to Host. */
static uint8_t m_ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; /**< Placeholder for received ACK payloads from Host. */


/**
 * @brief Function to read the button states.
 *
 * @return Returns states of buttons.
 */
static uint8_t input_get(void)
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

    return ~((uint8_t)((button3 << 3) | (button2 << 2) | (button1 << 1) | button0));
}


/**
 * @brief Function to control the LED outputs.
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


/**
 * @brief Initialize the BSP modules.
 */
static void ui_init(void)
{
    // Initialize application timer.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    // BSP initialization.
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 NULL);
    APP_ERROR_CHECK(err_code);

    // Set up logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Gazell ACK payload example. Device mode.\r\n");
    NRF_LOG_FLUSH();

    LEDS_ON(LEDS_MASK);
}


/*****************************************************************************/
/** @name Gazell callback function definitions  */
/*****************************************************************************/
/**
 * @brief TX success callback.
 *
 * @details If an ACK was received, another packet is sent.
 */
void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    bool     result_value         = false;
    uint32_t m_ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        result_value =
            nrf_gzll_fetch_packet_from_rx_fifo(pipe, m_ack_payload, &m_ack_payload_length);
        if (!result_value)
        {
            NRF_LOG_ERROR("RX fifo error \r\n");
            NRF_LOG_FLUSH();
        }

        if (m_ack_payload_length > 0)
        {
            output_present(m_ack_payload[0]);
        }
    }

    // Load data payload into the TX queue.
    m_data_payload[0] = input_get();

    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error \r\n");
        NRF_LOG_FLUSH();
    }
}


/**
 * @brief TX failed callback.
 *
 * @details If the transmission failed, send a new packet.
 *
 * @warning This callback does not occur by default since NRF_GZLL_DEFAULT_MAX_TX_ATTEMPTS
 * is 0 (inifinite retransmits).
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    NRF_LOG_ERROR("Gazell transmission failed\r\n");
    NRF_LOG_FLUSH();

    // Load data into TX queue.
    m_data_payload[0] = input_get();

    bool result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error \r\n");
        NRF_LOG_FLUSH();
    }
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_disabled()
{
}


/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main()
{
    // Set up the user interface (buttons and LEDs).
    ui_init();

    // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Attempt sending every packet up to MAX_TX_ATTEMPTS times.
    result_value = nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Load data into TX queue.
    m_data_payload[0] = input_get();

    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error \r\n");
        NRF_LOG_FLUSH();
    }

    // Enable Gazell to start sending over the air.
    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    while (1)
    {
        __WFE();
    }
}


