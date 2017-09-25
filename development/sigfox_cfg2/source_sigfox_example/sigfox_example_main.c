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
    // user data
    uint8_t test_data[SIGFOX_SEND_PAYLOAD_SIZE];
    uint8_t *p_down_link_data;

    //timer Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
    //sd init
    ble_stack_init_minimal();

    //snek mode enable
//    m_module_parameter.sigfox_snek_testmode_enable = 1;

    //set test send data
    sprintf((char*)test_data,"AABBCCDDEEFF");

    // create sigfox timer instance
    cfg_sigfox_timer_create();

    //Set the power level
    if(!cfg_sigfox_set_powerlevel(14))
    {
        cPrintLog(CDBG_MAIN_LOG, "ERROR SET POWER LEVEL");
    }
    //set RCZ
    sigfox_set_rcz(RCZ_1);
    sigfox_send_payload(test_data,&p_down_link_data);

    cPrintLog(CDBG_MAIN_LOG, "%s %d SIGFOX downlink data:%s, size:%d\n", __func__, __LINE__, (const char*)p_down_link_data, strlen((const char*)p_down_link_data));

    while(1)
    {
        sd_app_evt_wait();
    }
}
