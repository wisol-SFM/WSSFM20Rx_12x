/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @brief control acceleration senso.
 *
 * This file contains the control acceleration senso.
 */
 


#ifndef NRF_BLE_CFG_BMA250_H__
#define NRF_BLE_CFG_BMA250_H__

#include "cfg_board_def.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_twi.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
//#include "bma2x2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Structure for BMA250 state machine. */
typedef enum
{
	NONE_ACC,
	SET_S,
	SET_R,
	READ_DATA_S,
	READ_DATA_R,
	SLEEP_S,
	SLEEP_R,
	BYPASS_S,
	BYPASS_R,
	IO_SETUP,
	EXIT_ACC
} bma250_state_s;

struct bma_accel_data {
int16_t x,/**< accel x data 10,14 and 12 resolution*/
y,/**< accel y data 10,14 and 12 resolution*/
z;/**< accel z data 10,14 and 12 resolution*/
};

/**
 * @brief function for powering on acceleration sensor 
 */

void bma250_power_on(void);


/**
 * @brief function for creating  acceleration sensor timer.
 * 
 */

void cfg_bma250_timer_create(void);


/**
 * @brief function for starting  acceleration sensore timer.
 * 
 */

void cfg_bma250_timers_start(void);


/**
 * @brief function for stoping  acceleration sensore timer.
 * 
 */

void cfg_bma250_timers_stop(void);

/**
 * @brief function for getting  the current state from acceleration sensor.
 * 
 * @return current state from acceleration sensor
 */

bma250_state_s bma250_get_state(void);


/**
 * @brief function for setting  suspend mode in acceleration sensor.
 * 
 * @return NRF_SUCCESS or NRF_ERROR_TIMEOUT
 */
uint32_t bma250_req_suppend_mode(void);


/**@brief Function for setting sigfox state in acceleration sensor scheduler.
 *
 * @param[in] state  set the next state in acceleration sensor scheduler.
 *                         
 */

void bma250_set_state(bma250_state_s m_state);


/**@brief Function for indicating whether the current mode is test mode or not.
 *
 * @return true if the current mode is test mode, nor false
 *                         
 */

bool acc_is_bypass_mode(void);


/**@brief Function for sending the data  received from PC app to sigfox module 
 *
 * @param[in] byte sending one byte
 *                         
 */

void acc_bypass_put(const char * data, int size);

bool cfg_bma250_read_xyz(struct bma_accel_data *accel);

bool cfg_bma250_read_reg(uint8_t reg_addr,uint8_t * read);

bool cfg_bma250_write_reg(uint8_t reg_addr, uint8_t reg_data);
uint32_t cfg_bma250_ISR_pin_test(void);
uint32_t cfg_bma250_sw_reset(void);

void  bma250_read_chip(void);

#ifdef __cplusplus
}
#endif

#endif // NRF_BLE_CFG_MAIN_H__

/** @} */


