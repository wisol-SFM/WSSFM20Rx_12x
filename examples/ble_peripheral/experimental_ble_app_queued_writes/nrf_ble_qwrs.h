/* Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

 /** @cond To make doxygen skip this file */

#ifndef BLE_QWRS_H__
#define BLE_QWRS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_qwr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_BLE_QWRS_MAX_RCV_SIZE   128

/**@brief Queued Write Example Service event types. */
typedef enum
{
    BLE_QWRS_CHECK_RCVD_DATA,                        /* On this event, the application shall only verify if the data are correct.*/
    BLE_QWRS_NEW_DATA_RCVD,                          /* On this event, the application can act upon the received data*/
} nrf_ble_qwrs_evt_type;


/**@brief Queued Write Example Service event. */
typedef struct
{
    nrf_ble_qwrs_evt_type evt_type;                        //!< Type of event.
    uint16_t              rcv_length;
    uint8_t               rcvd_data[NRF_BLE_QWRS_MAX_RCV_SIZE];
} nrf_ble_qwrs_evt_t;


// Forward declaration of the nrf_ble_qwrs_t type.
struct nrf_ble_qwrs_t;


/**@brief Queued Write Example Service event handler type. returns a BLE_GATT_STATUS_CODES */
typedef uint16_t (*nrf_ble_qwrs_evt_handler_t) (struct nrf_ble_qwrs_t * p_qwrs,
                                                nrf_ble_qwrs_evt_t    * p_evt);


typedef struct
{
    nrf_ble_qwrs_evt_handler_t   evt_handler;       //!< Event handler to be called for handling events in the Queued Write Example  Service.
    ble_srv_error_handler_t      error_handler;     //!< Function to be called in case of an error.
    nrf_ble_qwr_t              * p_qwr_ctx;         //!< pointer to the initialized queued write context
} nrf_ble_qwrs_init_t;


typedef struct nrf_ble_qwrs_t
{
    uint8_t                    uuid_type;               //!< UUID type.
    uint16_t                   service_handle;          //!< Handle of Queued Write Example  Service (as provided by the BLE stack).
    uint16_t                   conn_handle;             //!< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).
    nrf_ble_qwrs_evt_handler_t evt_handler;             //!< Event handler to be called for handling events in the Queued Write Example  Service.
    ble_srv_error_handler_t    error_handler;           //!< Function to be called in case of an error.
    ble_gatts_char_handles_t   long_charact_handles;    //!< Handles related to the Queued Write Example long characteristic.
    ble_gatts_char_handles_t   charact_handles;         //!< Handles related to the Queued Write Example characteristic.
} nrf_ble_qwrs_t;


/**@brief Function for initializing the Queued Write Example Service.
 *
 * @details This call allows the application to initialize the Queued Write Example Service.
 *
 * @param[in]   p_qwrs_init  Information needed to initialize the service.
 * @param[out]  p_qwrs       Queued Write Example Service structure.

 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t nrf_ble_qwrs_init(nrf_ble_qwrs_init_t *p_qwrs_init, nrf_ble_qwrs_t *p_qwrs);


/**@brief event handler function for handling event from the queued write module.
 *
 * @param[in]  p_qwrs     Queued Write Example Service structure.
 * @param[in]  p_qwr      Queued Write structure.
 * @param[in]  p_evt      event received from the QWR module.
 *
 * @return      BLE_GATT_STATUS_SUCCESS if the received data are accepted, error code otherwise.
 */
uint16_t nrf_ble_qwrs_on_qwr_evt(nrf_ble_qwrs_t *p_qwrs,
                                 nrf_ble_qwr_t * p_qwr,
                                 nrf_ble_qwr_evt_t * p_evt);


#ifdef __cplusplus
}
#endif

#endif // BLE_QWRS_H__

/** @} */

/** @endcond */
