/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
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
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#if 0
#include "nrf_drv_timer.h"
const nrf_drv_timer_t MYTIMER = NRF_DRV_TIMER_INSTANCE(0);
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
}
#endif
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    while (true)
    {
        // Do nothing.
    }
}
/** @} */
