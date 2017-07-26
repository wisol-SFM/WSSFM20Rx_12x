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
 * @defgroup capacitive_sensor_example_main main.c
 * @{
 * @ingroup capacitive_sensor_example
 * @brief Capacitive Sensor Example main file.
 *
 * This file contains the source code for a sample application that uses a capacitive sensor.
 *
 */

#include <stdio.h>
#include <string.h>
#include "boards.h"
#include "nrf_csense.h"
#include "app_error.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"

#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* Time in RTC ticks between RTC interrupts. */
#define APP_TIMER_TICKS_TIMEOUT 2050

/* Timer initalization parameters. */
#define OP_QUEUES_SIZE          4
#define APP_TIMER_PRESCALER     0

/* Scale range. */
#define RANGE                   50

/* Analog inputs. */
#define AIN1                    1
#define AIN2                    2
#define AIN3                    3
#define AIN4                    4
#define AIN7                    7

/* Definition which pads use which analog inputs. */
#define BUTTON                  AIN7
#define PAD1                    AIN4
#define PAD4                    AIN4
#ifdef NRF51
#define PAD2                    AIN2
#define PAD3                    AIN3
#else
#define PAD2                    AIN1
#define PAD3                    AIN2
#endif

/* Threshold values for pads and button. */
#define THRESHOLD_PAD_1         1000
#define THRESHOLD_PAD_2         1000
#define THRESHOLD_PAD_3         1000
#define THRESHOLD_PAD_4         1000
#define THRESHOLD_BUTTON        1200

/*lint -e19 -save */
NRF_CSENSE_BUTTON_DEF(m_button, (BUTTON, THRESHOLD_BUTTON));
NRF_CSENSE_SLIDER_4_DEF(m_slider,
                        RANGE, 
                        (PAD1, THRESHOLD_PAD_1), 
                        (PAD2, THRESHOLD_PAD_2),
                        (PAD3, THRESHOLD_PAD_3), 
                        (PAD4, THRESHOLD_PAD_4));
/*lint -restore*/


/**
 * @brief Function for starting the internal LFCLK XTAL oscillator.
 *
 * Note that when using a SoftDevice, LFCLK is always on.
 *
 * @return Values returned by @ref nrf_drv_clock_init.
 */
static ret_code_t clock_config(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_clock_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    nrf_drv_clock_lfclk_request(NULL);

    return NRF_SUCCESS;
}

/**
 * @brief Function for handling slider interrupts.
 *
 * @param[in] step                          Detected step.
 */
static void slider_handler(uint16_t step)
{
    static uint16_t slider_val;
    if (slider_val != step)
    {
        NRF_LOG_INFO("Slider value: %03d.\r\n", step);
        slider_val = step;
    }
}

/**
 * @brief Event handler for Capacitive Sensor High module.
 *
 * @param [in] p_evt_type                    Pointer to event data structure.
 */
void nrf_csense_handler(nrf_csense_evt_t * p_evt)
{
    switch (p_evt->nrf_csense_evt_type)
    {
        case NRF_CSENSE_BTN_EVT_PRESSED:
            break;
        case NRF_CSENSE_BTN_EVT_RELEASED:
            if (p_evt->p_instance == (&m_button))
            {
                uint16_t * btn_cnt = ((uint16_t *)p_evt->p_instance->p_context);
                (*btn_cnt)++;
                NRF_LOG_INFO("Button touched %03d times.\r\n", (*btn_cnt));
            }
            break;
        case NRF_CSENSE_SLIDER_EVT_PRESSED:
        case NRF_CSENSE_SLIDER_EVT_RELEASED:
            break;
        case NRF_CSENSE_SLIDER_EVT_DRAGGED:
            if ((p_evt->p_instance == (&m_slider)) && (p_evt->params.slider.step != UINT16_MAX))
            {
                /*lint -e611 -save */
                ((void(*)(uint16_t, uint8_t))p_evt->p_instance->p_context)(p_evt->params.slider.step, 2);
                /*lint -restore*/
            }
            break;
        default:
            NRF_LOG_WARNING("Unknown event.\r\n");
            break;
    }
}

/**
 * @brief Function for starting Capacitive Sensor High module.
 *
 * Function enables one slider and one button.
 */
static void csense_start(void)
{
    ret_code_t err_code;

    static uint16_t touched_counter = 0;

    err_code = nrf_csense_init(nrf_csense_handler, APP_TIMER_TICKS_TIMEOUT);
    APP_ERROR_CHECK(err_code);

    nrf_csense_instance_context_set(&m_button, (void*)&touched_counter);
    nrf_csense_instance_context_set(&m_slider, (void*)slider_handler);

    err_code = nrf_csense_add(&m_button);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_csense_add(&m_slider);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Capacitive sensing library example.\r\n");

    APP_TIMER_INIT(APP_TIMER_PRESCALER, OP_QUEUES_SIZE, NULL);

    err_code = clock_config();
    APP_ERROR_CHECK(err_code);

    csense_start();

    while (1)
    {
        __WFI();
        NRF_LOG_FLUSH();
    }
}

/** @} */
