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
 * @defgroup nrf_drv_csense_example_main main.c
 * @{
 * @ingroup nrf_drv_csense_example
 * @brief Capacitive Sensor Example main file.
 *
 * This file contains the source code for a sample application that uses a capacitive sensor.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "nrf_drv_csense.h"
#include "app_error.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "app_uart.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* Time between RTC interrupts. */
#define APP_TIMER_TICKS_TIMEOUT 2000

/* Pin used to measure capacitor charging time. */
#ifdef NRF51
#define OUTPUT_PIN 30
#endif

/* Timer initialization parameters. */
#define OP_QUEUES_SIZE          4
#define APP_TIMER_PRESCALER     0

/* Analog inputs. */
#define AIN_1                   1
#define AIN_2                   2
#define AIN_7                   7

/* Masks of analog channels to be used by library. */
#ifdef NRF51
#define PAD_1_MASK              (1UL << AIN_2)
#else
#define PAD_1_MASK              (1UL << AIN_1)
#endif
#define PAD_2_MASK              (1UL << AIN_7)

#define PAD_ID_0                0
#define PAD_ID_1                1

#define UART_TX_BUF_SIZE 512                                                /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                                                  /**< UART RX buffer size. */


/* Threshold number of ticks (if COMP module present) or voltage in millivolts on sensor. */
volatile uint32_t threshold_value_pad1 = 850;
volatile uint32_t threshold_value_pad2 = 850;

/* Variables needed to properly configure threshold. */
volatile uint32_t max_value[2];
volatile uint32_t min_value[2];
bool conf_mode;

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**
 * @brief Function for uart initialization.
 */
ret_code_t uart_config(void)
{
    ret_code_t err_code;
    
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);
    APP_ERROR_CHECK(err_code);
    
    return err_code;
}
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

static inline void find_min_and_max(uint32_t val, uint8_t pad)
{
    if (val > max_value[pad])
    {
        max_value[pad] = val;
    }
    if (val < min_value[pad])
    {
        min_value[pad] = val;
    }
}

/**
 * @brief Capacitive sensor event handler.
 */
void csense_handler(nrf_drv_csense_evt_t * p_event_struct)
{
    switch (p_event_struct->analog_channel)
    {
        case AIN_1:
        case AIN_2:
            if (conf_mode)
            {
                printf("PAD1: %d.\r\n", p_event_struct->read_value);
                find_min_and_max(p_event_struct->read_value, PAD_ID_0);
                break;
            }
            if (p_event_struct->read_value >= threshold_value_pad1)
            {
                LEDS_ON(BSP_LED_0_MASK);
            }
            else
            {
                LEDS_OFF(BSP_LED_0_MASK);
            }
            break;
        case AIN_7:
            if (conf_mode)
            {
                printf("PAD2: %d.\r\n", p_event_struct->read_value);
                find_min_and_max(p_event_struct->read_value, PAD_ID_1);
                break;
            }
            if (p_event_struct->read_value >= threshold_value_pad2)
            {
                LEDS_ON(BSP_LED_2_MASK);
            }
            else
            {
                LEDS_OFF(BSP_LED_2_MASK);
            }
            break;
        default:
            break;
    }
}
/**
 * @brief Function for initializing the capacitive sensor.
 */
void csense_initialize(void)
{
    ret_code_t err_code;

    nrf_drv_csense_config_t csense_config = { 0 };

#ifdef NRF51
    csense_config.output_pin = OUTPUT_PIN;
#endif

    err_code = nrf_drv_csense_init(&csense_config, csense_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_csense_channels_enable(PAD_1_MASK | PAD_2_MASK);
}

/**
 * @brief Timer event handler for the capacitive sensor.
 *
 * @param[in] p_context             General purpose pointer. Will be passed to the time-out handler
 *                                  when the timer expires.
 *
 */
static void csense_timeout_handler(void * p_context)
{
    ret_code_t err_code;

    err_code = nrf_drv_csense_sample();
    if (err_code != NRF_SUCCESS)
    {
        printf("Busy.\r\n");
        return;
    }
}

/**
 * @brief Function for initalizing app timer.
 */
void start_app_timer(void)
{
    ret_code_t err_code;
        
    /* APP_TIMER definition for csense example. */
    APP_TIMER_DEF(timer_0);

    err_code = app_timer_create(&timer_0, APP_TIMER_MODE_REPEATED, csense_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(timer_0, APP_TIMER_TICKS_TIMEOUT, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for configuring pads threshold.
 */
void configure_thresholds(void)
{
    ret_code_t err_code;
    uint32_t new_th_pad_1;
    uint32_t new_th_pad_2;
    
    for (int i = 0; i < 2; i++)
    {
        max_value[i] = 0;
        min_value[i] = UINT32_MAX;
    }
    
    printf("Touch both pads.\r\n");
    nrf_delay_ms(1000);
    printf("3...\r\n");
    nrf_delay_ms(1000);
    printf("2...\r\n");
    nrf_delay_ms(1000);
    printf("1...\r\n");
    
    err_code = nrf_drv_csense_sample();   
    if (err_code != NRF_SUCCESS)
    {
        printf("Busy.\n");
        return;
    }
    while (nrf_drv_csense_is_busy());
    
    printf("Release both pads.\r\n");
    nrf_delay_ms(1000);
    printf("3...\r\n");
    nrf_delay_ms(1000);
    printf("2...\r\n");
    nrf_delay_ms(1000);
    printf("1...\r\n");
    
    err_code = nrf_drv_csense_sample();   
    if (err_code != NRF_SUCCESS)
    {
        printf("Busy.\n");
        return;
    }
    while (nrf_drv_csense_is_busy());

    nrf_delay_ms(100);    
    new_th_pad_1 = max_value[PAD_ID_0];
    new_th_pad_1 += min_value[PAD_ID_0];
    new_th_pad_1 /= 2;
    new_th_pad_2 = max_value[PAD_ID_1];
    new_th_pad_2 += min_value[PAD_ID_1];
    new_th_pad_2 /= 2;
    threshold_value_pad1 = new_th_pad_1;
    threshold_value_pad2 = new_th_pad_2;
    printf("New thresholds, AIN1: %d, AIN7: %d.\r\n", (unsigned int)new_th_pad_1,
                                                      (unsigned int)new_th_pad_2);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;
    char config;
    
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    err_code = uart_config();
    APP_ERROR_CHECK(err_code);

    APP_TIMER_INIT(APP_TIMER_PRESCALER, OP_QUEUES_SIZE, NULL);

    err_code = clock_config();
    APP_ERROR_CHECK(err_code);

    printf("Capacitive sensing library example.\r\n");
    
    csense_initialize();    

    printf("Do you want to enter configuration mode to set thresholds?(y/n)\r\n");
    scanf("%c", &config);
    
    conf_mode = (config == 'y') ? true : false;
    
    if (conf_mode)
    {
        configure_thresholds();
        conf_mode = false;
    }
    
    start_app_timer();    

    while (1)
    {
        __WFI();
    }
}

/** @} */
