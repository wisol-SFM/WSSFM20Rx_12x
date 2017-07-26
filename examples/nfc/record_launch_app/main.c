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


#include <nrf52.h>
#include <nrf52_bitfields.h>
#include <stdbool.h>
#include <stdint.h>
#include "nfc_t2t_lib.h"
#include "nfc_launchapp_msg.h"
#include "boards.h"
#include "nrf_error.h"
#include "app_error.h"
#include "hardfault.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/** @snippet [NFC Launch App usage_0] */
/* nRF Toolbox Android application package name */
static const uint8_t m_android_package_name[] = {'n', 'o', '.', 'n', 'o', 'r', 'd', 'i', 'c', 's',
                                                 'e', 'm', 'i', '.', 'a', 'n', 'd', 'r', 'o', 'i',
                                                 'd', '.', 'n', 'r', 'f', 't', 'o', 'o', 'l', 'b',
                                                 'o', 'x'};

/* nRF Toolbox application ID for Windows phone */
static const uint8_t m_windows_application_id[] = {'{', 'e', '1', '2', 'd', '2', 'd', 'a', '7', '-',
                                                   '4', '8', '8', '5', '-', '4', '0', '0', 'f', '-',
                                                   'b', 'c', 'd', '4', '-', '6', 'c', 'b', 'd', '5',
                                                   'b', '8', 'c', 'f', '6', '2', 'c', '}'};

uint8_t m_ndef_msg_buf[256];
/** @snippet [NFC Launch App usage_0] */


/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void * p_context, nfc_t2t_event_t event, const uint8_t * p_data, size_t data_length)
{
    (void)p_context;

    switch (event)
    {
        case NFC_T2T_EVENT_FIELD_ON:
            LEDS_ON(BSP_LED_0_MASK);
            break;

        case NFC_T2T_EVENT_FIELD_OFF:
            LEDS_OFF(BSP_LED_0_MASK);
            break;

        default:
            break;
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /** @snippet [NFC Launch App usage_1] */
    uint32_t len;
    uint32_t err_code;
    /** @snippet [NFC Launch App usage_1] */

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    /* Configure LED-pins as outputs */
    LEDS_CONFIGURE(BSP_LED_0_MASK);
    LEDS_OFF(BSP_LED_0_MASK);

    /* Set up NFC */
    err_code = nfc_t2t_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);

    /** @snippet [NFC Launch App usage_2] */
    /*  Provide information about available buffer size to encoding function. */
    len = sizeof(m_ndef_msg_buf);

    /* Encode launchapp message into buffer */
    err_code = nfc_launchapp_msg_encode(m_android_package_name,
                                        sizeof(m_android_package_name),
                                        m_windows_application_id,
                                        sizeof(m_windows_application_id),
                                        m_ndef_msg_buf,
                                        &len);

    APP_ERROR_CHECK(err_code);
    /** @snippet [NFC Launch App usage_2] */

    /* Set created message as the NFC payload */
    err_code = nfc_t2t_payload_set(m_ndef_msg_buf, len);
    APP_ERROR_CHECK(err_code);

    /* Start sensing NFC field */
    err_code = nfc_t2t_emulation_start();
    APP_ERROR_CHECK(err_code);

    while (1)
    {
        NRF_LOG_FLUSH();
    }
}


