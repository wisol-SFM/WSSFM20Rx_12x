/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup i2s_example_main main.c
 * @{
 * @ingroup i2s_example
 *
 * @brief I2S Example Application main file.
 *
 * This file contains the source code for a sample application using I2S.
 */

#include <stdio.h>
#include <string.h>
#include "nrf_drv_i2s.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define LED_MASK_OK         BSP_LED_0_MASK
#define LED_MASK_ERROR      BSP_LED_1_MASK

#define I2S_BUFFER_SIZE     1000
static uint32_t m_buffer_rx[I2S_BUFFER_SIZE];
static uint32_t m_buffer_tx[I2S_BUFFER_SIZE];

// Delay time between consecutive I2S transfers performed in the main loop
// (in milliseconds).
#define PAUSE_TIME          500
// Number of blocks of data (resulting from one call to data handler, thus
// having the size equal to half size of the buffer) to be contained in each
// transfer.
#define BLOCKS_TO_TRANSFER  20

static volatile uint8_t  m_blocks_transferred     = 0;
static          uint8_t  m_zero_samples_to_ignore = 0;
static          uint16_t m_sample_value_to_send;
static          uint16_t m_sample_value_expected;
static          bool     m_error_encountered;

static void prepare_tx_data(uint32_t * p_buffer, uint16_t number_of_words)
{
    // These variables will be both zero only at the very beginning of each
    // transfer, so we use them as the indication that the re-initialization
    // should be performed.
    if (m_blocks_transferred == 0 && m_zero_samples_to_ignore == 0)
    {
        // Number of initial samples (actually pairs of L/R samples) with zero
        // values that should be ignored - see the comment in 'check_samples'.
        m_zero_samples_to_ignore = 2;
        m_sample_value_to_send   = 0xCAFE;
        m_sample_value_expected  = 0xCAFE;
        m_error_encountered      = false;
    }

    // [each data word contains two 16-bit samples]
    uint16_t i;
    for (i = 0; i < number_of_words; ++i)
    {
        uint16_t sample_l = m_sample_value_to_send - 1;
        uint16_t sample_r = m_sample_value_to_send + 1;
        ++m_sample_value_to_send;

        ((uint16_t *)p_buffer)[2 * i]     = sample_l;
        ((uint16_t *)p_buffer)[2 * i + 1] = sample_r;
    }
}


static bool check_samples(uint32_t const * p_buffer, uint16_t number_of_words)
{


    // [each data word contains two 16-bit samples]
    uint16_t i;
    for (i = 0; i < number_of_words; ++i)
    {
        uint16_t actual_sample_l   = ((uint16_t const *)p_buffer)[2 * i];
        uint16_t actual_sample_r   = ((uint16_t const *)p_buffer)[2 * i + 1];

        // Normally a couple of initial samples sent by the I2S peripheral
        // will have zero values, because it starts to output the clock
        // before the actual data is fetched by EasyDMA. As we are dealing
        // with streaming the initial zero samples can be simply ignored.
        if (m_zero_samples_to_ignore > 0 &&
            actual_sample_l == 0 &&
            actual_sample_r == 0)
        {
            --m_zero_samples_to_ignore;
        }
        else
        {
            m_zero_samples_to_ignore = 0;

            uint16_t expected_sample_l = m_sample_value_expected - 1;
            uint16_t expected_sample_r = m_sample_value_expected + 1;
            ++m_sample_value_expected;

            if (actual_sample_l != expected_sample_l ||
                actual_sample_r != expected_sample_r)
            {
                NRF_LOG_INFO("%3u: %04x/%04x, expected: %04x/%04x\r\n",
                    m_blocks_transferred, actual_sample_l, actual_sample_r,
                    expected_sample_l, expected_sample_r);
                return false;
            }
        }
    }
    NRF_LOG_INFO("%3u: OK\r\n", m_blocks_transferred);
    return true;
}


static void check_rx_data(uint32_t const * p_buffer, uint16_t number_of_words)
{
    ++m_blocks_transferred;

    if (!m_error_encountered)
    {
        m_error_encountered = !check_samples(p_buffer, number_of_words);
    }

    if (m_error_encountered)
    {
        LEDS_OFF(LED_MASK_OK);
        LEDS_INVERT(LED_MASK_ERROR);
    }
    else
    {
        LEDS_OFF(LED_MASK_ERROR);
        LEDS_INVERT(LED_MASK_OK);
    }
}


// This is the I2S data handler - all data exchange related to the I2S transfers
// is done here.
static void data_handler(uint32_t const * p_data_received,
                         uint32_t       * p_data_to_send,
                         uint16_t         number_of_words)
{
    // Non-NULL value in 'p_data_received' indicates that a new portion of
    // data has been received and should be processed.
    if (p_data_received != NULL)
    {
        // In this example we simply check if received data corresponds to what
        // we send.
        check_rx_data(p_data_received, number_of_words);
    }

    // Non-NULL value in 'p_data_to_send' indicates that the driver needs
    // a new portion of data to send.
    if (p_data_to_send != NULL)
    {
        // Here we write the provided buffer with consecutive 16-bit values
        // in order to be able to check later if they were transmitted properly.
        prepare_tx_data(p_data_to_send, number_of_words);
    }
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    #ifdef DEBUG
    app_error_print(id, pc, info);
    #endif

    LEDS_ON(LEDS_MASK);
    while (1);
}


int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    LEDS_CONFIGURE(LED_MASK_OK | LED_MASK_ERROR);

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("\r\n"
           "I2S loopback example\r\n");

    nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;
    // In Master mode the MCK frequency and the MCK/LRCK ratio should be
    // set properly in order to achieve desired audio sample rate (which
    // is equivalent to the LRCK frequency).
    // For the following settings we'll get the LRCK frequency equal to
    // 15873 Hz (the closest one to 16 kHz that is possible to achieve).
    config.sdin_pin  = I2S_SDIN_PIN;
    config.sdout_pin = I2S_SDOUT_PIN;
    config.mck_setup = NRF_I2S_MCK_32MDIV21;
    config.ratio     = NRF_I2S_RATIO_96X;
    config.channels  = NRF_I2S_CHANNELS_STEREO;
    err_code = nrf_drv_i2s_init(&config, data_handler);
    APP_ERROR_CHECK(err_code);

    for (;;)
    {
        memset(m_buffer_rx, 0xCC, sizeof(m_buffer_rx));

        m_blocks_transferred = 0;

        err_code = nrf_drv_i2s_start(m_buffer_rx, m_buffer_tx,
            I2S_BUFFER_SIZE, 0);
        APP_ERROR_CHECK(err_code);

        while (m_blocks_transferred < BLOCKS_TO_TRANSFER)
        {}
        nrf_drv_i2s_stop();

        LEDS_OFF(LED_MASK_OK | LED_MASK_ERROR);
        nrf_delay_ms(PAUSE_TIME);

        NRF_LOG_FLUSH();
    }
}

/** @} */
