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
 * This project requires that a device that runs the
 * @ref gzll_device_m_ack_payload_example is used as a counterpart for
 * receiving the data. This can be on either an nRF5x device or an nRF24Lxx device
 * running the \b gzll_device_m_ack_payload example in the nRFgo SDK.
 *
 * This example listens for a packet and sends an ACK
 * when a packet is received. The contents of the first payload byte of
 * the received packet is output on the GPIO Port BUTTONS.
 * The contents of GPIO Port LEDS are sent in the first payload byte (byte 0)
 * of the ACK packet.
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
/** @name Configuration  */
/*****************************************************************************/
#define PIPE_NUMBER             0  ///< Pipe 0 is used in this example.

#define TX_PAYLOAD_LENGTH       1  ///< 1-byte payload length is used when transmitting.

#define APP_TIMER_PRESCALER     0  ///< Value of the RTC PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE 8u ///< Size of timer operation queues.


static uint8_t m_data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Placeholder for data payload received from host.
static uint8_t m_ack_payload[TX_PAYLOAD_LENGTH];                  ///< Payload to attach to ACK sent to device.

extern nrf_gzll_error_code_t nrf_gzll_error_code;                 ///< Error code.


/**
 * @brief Function to read the button state.
 *
 * @return Returns states of the buttons.
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

    // Set up logger.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Gazell ACK payload example. Host mode.\r\n");
    NRF_LOG_FLUSH();

    LEDS_ON(LEDS_MASK);
}


/*****************************************************************************/
/** @name Gazell callback function definitions.  */
/*****************************************************************************/
/**
 * @brief RX data ready callback.
 *
 * @details If a data packet was received, the first byte is written to LEDS.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    // Pop packet and write first byte of the payload to the GPIO port.
    bool result_value = nrf_gzll_fetch_packet_from_rx_fifo(pipe,
                                                           m_data_payload,
                                                           &data_payload_length);

    if (!result_value)
    {
        NRF_LOG_ERROR("RX fifo error \r\n");
        NRF_LOG_FLUSH();
    }

    if (data_payload_length > 0)
    {
        output_present(m_data_payload[0]);
    }

    // Read buttons and load ACK payload into TX queue.
    m_ack_payload[0] = input_get(); // Button logic is inverted.

    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_ack_payload, TX_PAYLOAD_LENGTH);
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
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
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
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main()
{
    // Set up the user interface.
    ui_init();

    // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Load data into TX queue.
    m_ack_payload[0] = input_get();

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


