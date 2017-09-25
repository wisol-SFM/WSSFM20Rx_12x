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

static void main_schedule_timeout_handler_examples(void * p_context)
{
    main_schedule_tick++;
}

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif
#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

APP_TIMER_DEF(m_main_timer_id);

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
        case module_parameter_item_snek_testmode_enable:
            ret = m_module_parameter.sigfox_snek_testmode_enable;
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

bool module_parameter_get_bootmode(int *bootmode)
{
    return false;
}
bool module_parameter_erase_and_reset(void)
{
    return false;
}

void module_parameter_check_update(void)
{
    if(m_module_parameter_update_req)
    {
        m_module_parameter_update_req = false;
    }
    return;
}

void main_examples_prepare(void)
{
    return;
}

int main(void)
{
    volatile uint32_t err_code;
    int wifi_result;
    uint32_t get_cnt;
    uint8_t *ssid;
    int32_t *rssi;
    uint8_t *bssid;
    int i;

    //timer Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
    //sd init
    ble_stack_init_minimal();

    //main tick timer init (optional)
    err_code = app_timer_create(&m_main_timer_id, APP_TIMER_MODE_REPEATED, main_schedule_timeout_handler_examples);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(APP_MAIN_SCHEDULE_MS, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

    //disable wifi info log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));

    //Initalize resource for WIFI module 
    wifi_drv_init();

    set_scan_interval(10);
    wifi_result = start_AP_scan();

    if(wifi_result == CWIFI_Result_OK)
    {
        wifi_result = get_AP_scanResult(&get_cnt, &ssid, &rssi, &bssid);
        if(wifi_result == CWIFI_Result_OK)
        {
            cPrintLog(CDBG_MAIN_LOG, "AP_scanResult ok! ap cnt: %d\n", get_cnt);
            for(i=0; i<get_cnt; i++)
            {
                cPrintLog(CDBG_MAIN_LOG, "%d : %s %d %02x:%02x:%02x:%02x:%02x:%02x\n", i+1, &ssid[CWIFI_SSID_SIZE*i], rssi[i], 
                    bssid[(CWIFI_BSSID_SIZE*i)+0], bssid[(CWIFI_BSSID_SIZE*i)+1], bssid[(CWIFI_BSSID_SIZE*i)+2], 
                    bssid[(CWIFI_BSSID_SIZE*i)+3], bssid[(CWIFI_BSSID_SIZE*i)+4], bssid[(CWIFI_BSSID_SIZE*i)+5] );
            }
        }
        else if(wifi_result == CWIFI_Result_NoData)
        {
            cPrintLog(CDBG_MAIN_LOG, "WIFI MODULE NoData!\n");
        }
        else
        {
            cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
        }
    }
    else
    {
        cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
    }

    while(1)
    {
        sd_app_evt_wait();
    }

}
