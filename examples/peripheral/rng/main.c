/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup rng_example_main main.c
 * @{
 * @ingroup rng_example
 * @brief Random Number Generator Example Application main file.
 *
 */


#include <stdio.h>
#include <stdint.h>
#include "bsp.h"
#include "nrf_delay.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_rng.h"
#include "nrf_assert.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#ifdef SOFTDEVICE_PRESENT
#include "softdevice_handler.h"
#endif // SOFTDEVICE_PRESENT

#define RANDOM_BUFF_SIZE 16                                                           /**< Random numbers buffer size. */

void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name)
{
    /* empty function - needed by softdevice handler */
}

/** @brief Function for getting vector of random numbers.
 *
 * @param[out] p_buff                               Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length                               Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */
uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size)
{
    uint8_t available;
    uint32_t err_code;
    err_code = nrf_drv_rng_bytes_available(&available);
    APP_ERROR_CHECK(err_code);
    uint8_t length = (size<available) ? size : available;
    err_code = nrf_drv_rng_rand(p_buff,length);
    APP_ERROR_CHECK(err_code);
    return length;
}

/** @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

#ifdef SOFTDEVICE_PRESENT
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
#endif // SOFTDEVICE_PRESENT

    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        uint8_t p_buff[RANDOM_BUFF_SIZE];
        uint8_t length = random_vector_generate(p_buff,RANDOM_BUFF_SIZE);
        NRF_LOG_INFO("Random Vector:\r\n");
        NRF_LOG_HEXDUMP_INFO(p_buff, length);
        NRF_LOG_INFO("\r\n");
        nrf_delay_ms(100);
        NRF_LOG_FLUSH();
    }
}


/** @} */
