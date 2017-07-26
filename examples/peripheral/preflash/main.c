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
 *
 * @defgroup led_softblink_example_main main.c
 * @{
 * @ingroup led_softblink_example
 * @brief LED Soft Blink Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "led_softblink.h"
#include "app_error.h"
#include "sdk_errors.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "nfc_fixes.h"
#include "nfc_t2t_lib.h"
#include "nfc_uri_msg.h"
#include "nrf_delay.h"

/*Timer initalization parameters*/
#define APP_TIMER_OP_QUEUE_SIZE     3
#define APP_TIMER_PRESCALER         0

 //URL "nordicsemi.com/start52dk"
static const uint8_t m_url[] = {'n', 'o', 'r', 'd', 'i', 'c', 's', 'e', 'm', 'i', '.',
                                'c', 'o', 'm','/','s','t','a','r','t','5','2','d','k'};

uint8_t             m_ndef_msg_buf[256];        ///< Buffer for the NFC NDEF message.
volatile uint32_t   m_active_led_mask;          ///< LED mask.
volatile bool       m_update_softblink = false; ///< Flag for signaling a change of the LED mask for the softbling engine.

/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void * p_context, nfc_t2t_event_t event, const uint8_t * p_data, size_t data_length)
{
    (void)p_context;

    switch (event)
    {
        case NFC_T2T_EVENT_FIELD_ON:
            (void)led_softblink_stop();
            nrf_gpio_pins_clear(LEDS_MASK);
            break;
        case NFC_T2T_EVENT_FIELD_OFF:
            nrf_gpio_pins_set(LEDS_MASK);
            (void)led_softblink_start(m_active_led_mask);
            break;
        case NFC_T2T_EVENT_DATA_READ:
            break;
        default:
            break;
    }
}

static void nfc_init(void)
{
    ret_code_t err_code;
    uint32_t   len = sizeof(m_ndef_msg_buf);

    /* Set up NFC */
    err_code = nfc_t2t_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);

    /* Provide information about available buffer size to encoding function */

    /* Encode URI message into buffer */
    err_code = nfc_uri_msg_encode(NFC_URI_HTTP_WWW,
                                  m_url,
                                  sizeof(m_url),
                                  m_ndef_msg_buf,
                                  &len);

    APP_ERROR_CHECK(err_code);

    /* Set created message as the NFC payload */
    err_code = nfc_t2t_payload_set(m_ndef_msg_buf, len);
    APP_ERROR_CHECK(err_code);

    /* Start sensing NFC field */
    err_code = nfc_t2t_emulation_start();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for LEDs initialization.
 */
static void leds_init(void)
{
    ret_code_t err_code;

    led_sb_init_params_t led_sb_init_params = LED_SB_INIT_DEFAULT_PARAMS(LEDS_MASK);
    led_sb_init_params.off_time_ticks       = 32768;
    led_sb_init_params.on_time_ticks        = 16384;
    led_sb_init_params.duty_cycle_max       = 200;
    led_sb_init_params.duty_cycle_min       = 4;
    led_sb_init_params.duty_cycle_step      = 1;

    err_code = led_softblink_init(&led_sb_init_params);
    APP_ERROR_CHECK(err_code);
}

static void clock_init(void)
{
    uint32_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            m_active_led_mask = BSP_LED_0_MASK;
            break;
        case BSP_EVENT_KEY_1:
            m_active_led_mask = BSP_LED_1_MASK;
            break;
        case BSP_EVENT_KEY_2:
            m_active_led_mask = BSP_LED_2_MASK;
            break;
        case BSP_EVENT_KEY_3:
            m_active_led_mask = BSP_LED_3_MASK;
            break;
        default:
            return; // no implementation needed
    }

    m_update_softblink = true; // request update of blinked LED
}

/**
 * @brief Function for updating LED to be softly blinking.
 */
static void softblink_led_update(void)
{
    uint32_t err_code;

    if (m_update_softblink == false)
    {
         // nothing to do
         return;
    }

    m_update_softblink = false;

    err_code = led_softblink_stop();
    APP_ERROR_CHECK(err_code);

    err_code = led_softblink_start(m_active_led_mask);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    clock_init();

    // Start APP_TIMER to generate timeouts.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    leds_init();

    err_code = bsp_init(BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_evt_handler);
    APP_ERROR_CHECK(err_code);

    nfc_init();

    m_active_led_mask = BSP_LED_0_MASK;
    err_code = led_softblink_start(m_active_led_mask);
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        __WFE();
        softblink_led_update();
    }
}

/** @} */
