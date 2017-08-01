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

/** @file
 *
 * @defgroup ble_sdk_srv_cgms_meas Continuous Glucose Monitoring Service Measurement
 * @{
 * @ingroup ble_cgms
 * @brief Continuous Glucose Monitoring Service Measurement module.
 *
 * @details This module implements parts of the Continuous Glucose Monitoring that relate to the
 *          Measurement characteristic. Events are propagated to this module from @ref ble_cgms
 *          using @ref cgms_meas_on_write.
 *
 */


#ifndef NRF_BLE_CFG_MAIN_H__
#define NRF_BLE_CFG_MAIN_H__
#ifdef __cplusplus
extern "C" {
#endif
void main_examples_prepare(void);
#ifdef __cplusplus
}
#endif

#endif // NRF_BLE_CFG_MAIN_H__

/** @} */
