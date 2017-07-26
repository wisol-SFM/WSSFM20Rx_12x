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
 * @brief tracking Sample Application main file.
 *
 * This file contains the source code for an tracking sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "cfg_board_def.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_flash.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "peer_manager.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "cfg_app_main.h"
#include "cfg_sigfox_module.h"
#include "cfg_bma250_module.h"
#include "cfg_tmp102_module.h"
#include "cfg_gps_module.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"
#include "cfg_board.h"
#include "nrf_drv_gpiote.h"
#include "fstorage.h"
#include "fds.h"
#include "nrf_drv_twi.h"
#include "nfc_t2t_lib.h"
#include "nfc_uri_msg.h"
#include "nfc_launchapp_msg.h"
#include "hardfault.h"
#include "ble_dfu.h"
#include "ble_nus.h"
#include "nrf_drv_clock.h"
#include "cfg_external_sense_gpio.h"

unsigned int main_schedule_tick = 0;
volatile bool main_wakeup_interrupt;
#ifdef CDEV_NUS_MODULE
nus_service_parameter_t m_nus_service_parameter;
#endif
module_peripheral_data_t m_module_peripheral_data;
module_peripheral_ID_t m_module_peripheral_ID;  //id values (ble mac, sigfox id, wifi mac ...)
module_parameter_t m_module_parameter;  //setting values
bool m_module_parameter_update_req;
uint8_t   avg_report_volts;

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif
#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define FEATURE_GPS_NMEA_LOG_ONOFF

static void ble_stack_init_minimal(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC_250_PPM;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    // Enable SoftDevice stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

unsigned int main_get_param_val(module_parameter_item_e item)
{
    unsigned int ret = 0;
    switch(item)
    {
        case module_parameter_item_gps_tracking_time_sec:
            ret = m_module_parameter.gps_acquire_tracking_time_sec;
            break;

        default:
            break;
    }
    return ret;
}

void main_set_param_val(module_parameter_item_e item, unsigned int val)
{
    return;
}

bool main_check_ctrl_mode_allowed_state(void)
{
    return false;
}

bool main_work_mode_change_request(cfg_board_work_mode_e mode)
{
    return false;
}

cfg_board_work_mode_e main_work_mode_get_cur(void)
{
    return cfg_board_work_normal;
}

int main(void)
{
    int result = 0;
    unsigned int tracking_time = 0;
    unsigned int gps_sec = 0;
    char *ns;
    char *latitude;
    char *ew;
    char *longitude;
	
//timer Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
//sd init
    ble_stack_init_minimal();

// Initalize for GPS module(initializing gpio of gps)
    gps_init();

//set gps tracking interval time
    gps_sec = 60; // sec
    gps_tracking_set_interval(module_parameter_item_gps_tracking_time_sec, gps_sec);

//get gps tracking timeout
    tracking_time = gps_tracking_get_interval(module_parameter_item_gps_tracking_time_sec);
    cPrintLog(CDBG_MAIN_LOG, "GPS get Tracking time[%d] \n", tracking_time);

// set enable/disable of gps C/N0 check(save current consumption)
    set_cn0_current_savetime_enable(module_parameter_item_gps_cn0_current_savetime_enable, CGPS_CNO_CHECK_DISABLE);
    
// nmea data request of gps tracking
    result = start_gps_tracking();

#ifdef FEATURE_GPS_NMEA_LOG_ONOFF
    while(1)
    {
        nrf_delay_ms(200);
        cPrintLog(CDBG_GPS_INFO, "LINE[%d]  ==================================================== --[[    \n",__LINE__);
        cDataDumpPrintOut(CDBG_MAIN_LOG, m_cGpsRxNMEA_Buf, CGPS_SPI_BUF_SIZE);
        cPrintLog(CDBG_MAIN_LOG, "[%d]  size[%d]  m_cGpsNMEABuf[%s]  \n",__LINE__, sizeof(m_cGpsRxNMEA_Buf), m_cGpsRxNMEA_Buf);
        cPrintLog(CDBG_GPS_INFO, "LINE[%d]  ==================================================== --]]    \n",__LINE__);

        if (cGps_nmea_position_fix_check() == CGPS_Result_OK)
        {
            break;
        }
    }
#endif

    result = start_gps_nmea_data_check();
    if(result == CGPS_Result_OK)
    {
// get last location (gps position fixed)
        result = get_lastLocation(&ns, &latitude, &ew, &longitude);
        if(result == CGPS_Result_OK)
        {
// success of get gps data
            cPrintLog(CDBG_MAIN_LOG, "GPS Position NS[%s]\n", ns);
            cPrintLog(CDBG_MAIN_LOG, "GPS Position Latitude[%s]\n", latitude);
            cPrintLog(CDBG_MAIN_LOG, "GPS Position EW[%s]\n", ew);
            cPrintLog(CDBG_MAIN_LOG, "GPS Position Longitude[%s]\n", longitude);
        }
        else
        {
// no gps data
            cPrintLog(CDBG_MAIN_LOG, "GPS Module NoData!\n");
        }
    }
    else if(result == CGPS_Result_NoData)
    {
// no gps data
        cPrintLog(CDBG_MAIN_LOG, "GPS Module NoData!\n");
    }
    else if(result == CGPS_Result_NotStarted)
    {
// gps C/N0 dB-Hz check
        cPrintLog(CDBG_MAIN_LOG, "GPS C/N0 dB-Hz Low!\n");
    }
    else if(result == CGPS_Result_Fix_Fail)
    {
// position fix fail
        cPrintLog(CDBG_MAIN_LOG, "GPS Tracking Fail!\n");
    }
    else
    {
// not available GPS
        cPrintLog(CDBG_MAIN_LOG, "Not Available GPS Module!\n");
    }

    while(1)
    {
        sd_app_evt_wait();
    }
}
