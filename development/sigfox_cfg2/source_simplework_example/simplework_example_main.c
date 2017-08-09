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

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif
#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define APP_BEACON_INFO_LENGTH          0x11

APP_TIMER_DEF(main_wakeup_timer_id);
APP_TIMER_DEF(main_sec_tick_timer_id);

module_parameter_t m_module_parameter;  //setting values
nus_service_parameter_t m_nus_service_parameter;
module_peripheral_data_t m_module_peripheral_data;
module_peripheral_ID_t m_module_peripheral_ID;
bool m_module_parameter_update_req;
uint8_t   avg_report_volts;

bool main_wakeup_evt_expired = false;
uint8_t   m_main_sec_tick;

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH];

unsigned int main_get_param_val(module_parameter_item_e item)
{
    unsigned int ret = 0;
    switch(item)
    {
        case module_parameter_item_snek_testmode_enable:
            ret = m_module_parameter.sigfox_snek_testmode_enable;
            break;

        case module_parameter_item_gps_tracking_time_sec:
            ret = m_module_parameter.gps_acquire_tracking_time_sec;
            break;

        default:
            break;
    }
    return ret;
}

void module_parameter_check_update(void)
{
    if(m_module_parameter_update_req)
    {
        m_module_parameter_update_req = false;
    }
    return;
}

void main_set_param_val(module_parameter_item_e item, unsigned int val)
{
    return;
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            cPrintLog(CDBG_MAIN_LOG, "BLE_GAP_EVT_DISCONNECTED.\r\n");
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
            cPrintLog(CDBG_MAIN_LOG, "BLE_GAP_EVT_CONNECTED.\r\n");
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            cPrintLog(CDBG_MAIN_LOG, "GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            cPrintLog(CDBG_MAIN_LOG, "GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t event)
{
    fs_sys_event_handler(event);
    ble_advertising_on_sys_evt(event);
}

static void ble_stack_init(void)
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

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;

        case BLE_ADV_EVT_IDLE:
            break;

        default:
            break;
    }
}

static void gap_params_init(const char *payload, int payload_size)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    unsigned int index = 0;

    memset(m_beacon_info, 0, sizeof(m_beacon_info));

    if(payload)
    {
        memcpy(m_beacon_info, payload, payload_size);
        index = payload_size;
    }
    else
    {
        strcpy((char*)&m_beacon_info[index], "empty");
        index = strlen((char*)m_beacon_info);
    }
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_beacon_info,
                                          index);
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = (uint16_t)(MSEC_TO_UNITS(15, UNIT_1_25_MS));
    gap_conn_params.max_conn_interval = (uint16_t)(MSEC_TO_UNITS(100, UNIT_1_25_MS));
    gap_conn_params.slave_latency     = 0;
    gap_conn_params.conn_sup_timeout  = (4 * 100);  // 4 seconds

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(4);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = MSEC_TO_UNITS(200, UNIT_0_625_MS);  //200ms
    options.ble_adv_fast_timeout = 60;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void Ble_set_payload(const char *name)
{
    gap_params_init(name, strlen(name));
    advertising_init();
}


static void main_sec_tick_timer_handler(void * p_context)
{
    m_main_sec_tick++;
}

static void main_sec_tick_timer_init(void)
{
    uint32_t err_code;

    err_code = app_timer_create(&main_sec_tick_timer_id, APP_TIMER_MODE_REPEATED, main_sec_tick_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(main_sec_tick_timer_id, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

}

static void main_wakeup_timer_timeout_handler(void * p_context)
{
    cPrintLog(CDBG_MAIN_LOG, "Wakeup timer expired\n");
    main_wakeup_evt_expired = true;
}

static void main_wakeup_timer_init(void)
{
    uint32_t err_code;
    err_code = app_timer_create(&main_wakeup_timer_id, APP_TIMER_MODE_REPEATED, main_wakeup_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);

}
static void main_wakeup_timer_start(int sec)
{
    uint32_t err_code;
    uint32_t timeout_ticks; 

    timeout_ticks = APP_TIMER_TICKS((sec*1000), APP_TIMER_PRESCALER);
    cPrintLog(CDBG_MAIN_LOG, "Wakeup start ticks:%d\n", timeout_ticks);
    err_code = app_timer_start(main_wakeup_timer_id, timeout_ticks, NULL);  //timer to wake up every 600s
    APP_ERROR_CHECK(err_code);

}

static void main_resource_init(void)
{
    //timer Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    ble_stack_init();
    main_wakeup_timer_init();
    main_sec_tick_timer_init();
}

static void Init_sigfox(void)
{
//    m_module_parameter.sigfox_snek_testmode_enable = true;  //snek test mode
    cfg_sigfox_prepare_start();
}

static void Init_wifi(void)
{
    wifi_drv_init();
}

static void Init_ble(void)
{
    gap_params_init(NULL, 0);
    advertising_init();
}

static void Init_gps(void)
{
    gps_init();
    set_cn0_current_savetime_enable(module_parameter_item_gps_cn0_current_savetime_enable, CGPS_CNO_CHECK_DISABLE);
}

static void init_module(void)
{   
    main_resource_init();
    Init_sigfox();
    Init_wifi();
    Init_ble();
    Init_gps();
}

static void Sigfox_set_rcz(sigfox_rcz RCZ)
{
    sigfox_set_rcz(RCZ);
}

static void Sigfox_set_tx_power(int pwr)
{
    cfg_sigfox_set_powerlevel(pwr);
}

static void Wifi_set_scan_time(int time)
{
    set_scan_interval(time);
}

static void Gps_set_scan_time(int time)
{
    gps_tracking_set_interval(module_parameter_item_gps_tracking_time_sec, time);
}

static void Wifi_get_scanned_BSSID(unsigned char *bssid_buf)
{
    uint32_t get_cnt;
    uint8_t *ssid;
    int32_t *rssi;
    uint8_t *bssid;
    
    memset(bssid_buf, 12, 0);
    if(start_AP_scan() == CWIFI_Result_OK)
    {
        get_AP_scanResult(&get_cnt, &ssid, &rssi, &bssid);
        memcpy(bssid_buf, bssid, 12);
    }
}

static void Sigfox_send_payload(unsigned char *payload)
{
    sigfox_send_payload(payload, NULL);
}

static bool gps_acquire(unsigned char *position)
{
    uint8_t *pGpsInfo;
    
    memset(position, 12, 0);
    if(start_gps_tracking() == CGPS_Result_OK)
    {
        if(cGps_nmea_get_bufPtr(&pGpsInfo) == CGPS_Result_OK)
        {
            memcpy(position, pGpsInfo, 12);
            return true;
        }
    }

    while(cGps_bus_busy_check());  //wait for release gps spi 
    return false;
}

void Ble_start_beacon(void)
{
    ble_advertising_start(BLE_ADV_MODE_FAST);
}

int main(void)
{
    unsigned char bssid[12];
    unsigned char position[12];

    cPrintLog(CDBG_MAIN_LOG, "\nStart simple example\n");
    
    init_module();  // all init functions could be embedded into a init_module() function
    
    Sigfox_set_rcz(RCZ_1);
    Sigfox_set_tx_power(15);
    Wifi_set_scan_time(10);
    Gps_set_scan_time(60);
    Ble_set_payload("testpayload");
    main_wakeup_timer_start(600);  // Here you shoud set up a timer to wake up every 600s

    while(1)
    {
        Wifi_get_scanned_BSSID(bssid);
        Sigfox_send_payload(bssid);
        if(gps_acquire(position) == true)
        { 
            Sigfox_send_payload(position);
        }
        Ble_start_beacon();

        // Here you wait for the timer event.
        while(1)
        {
            if(main_wakeup_evt_expired)
            {
                main_wakeup_evt_expired=false;
                break;
            }
            sd_app_evt_wait();
        }
    }
}
