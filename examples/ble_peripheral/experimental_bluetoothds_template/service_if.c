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
 */

#include "service_if.h"
#include <stdint.h>
#include "nrf_log.h"


/* Static variables for each service will be generated here
//static ble_dummy_service_t       m_dummy;


// One handler function per service will be generated for handling service events.
static void on_rns_evt(ble_dummy_service_t     * p_dummy_service,
                       ble_dummy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_dummy_SERVICE_EVT_NOTIFICATION_ENABLED:
            NRF_LOG_INFO("[Bluetooth_IF]: DUMMY evt NOTIFICATION_ENABLED. \r\n");
            break;
        case BLE_dummy_SERVICE_EVT_NOTIFICATION_DISABLED:
            NRF_LOG_INFO("[Bluetooth_IF]: DUMMY evt NOTIFICATION_DISABLED. \r\n");
            break;
        case BLE_dummy_SERVICE_dummy_EVT_CCCD_WRITE:
            NRF_LOG_INFO("[Bluetooth_IF]: DUMMY evt CCCD_WRITE.\r\n");
            break;
        default:
            // No implementation needed.
            break;
    }
}
*/

// Init function will have generated one init block per service
uint32_t bluetooth_init(void)
{
    return NRF_SUCCESS;
}

// Generic handler function for ble events will call each service's individual on_ble_evt function
void bluetooth_on_ble_evt(ble_evt_t * p_ble_evt)
{

}
