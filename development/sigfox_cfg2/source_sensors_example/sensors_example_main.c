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

int16_t tmp102_int, tmp102_dec;

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

void main_wakeup_interrupt_set(void)
{
    main_wakeup_interrupt = true;
}

void accelerometer_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    main_wakeup_interrupt_set();
    cPrintLog(CDBG_MAIN_LOG,"%s", "g-sensor shaking\n");
}

void accelerometer_interrupt_init(void)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_gpiote_in_init(PIN_DEF_ACC_INT1, &in_config, accelerometer_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_DEF_ACC_INT1, true);
}

int main(void)
{
    int result =0;
    uint32_t threshold_low_result = 0, threshold_high_result = 0 ;
        
    //timer Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
    //sd init
    ble_stack_init_minimal();

    //magnetic sensor 
    cfg_magnetic_sensor_init(main_wakeup_interrupt_set, NULL);

    //gpio key init
    cfg_wkup_gpio_init(main_wakeup_interrupt_set);

    //accelerometer interrupt init
    cfg_i2c_master_init();
    bma250_read_chip();
    cfg_bma250_timer_create();
    bma250_set_state(NONE_ACC);
    if(bma250_get_state() == NONE_ACC)
    {
        cPrintLog(CDBG_MAIN_LOG, "%s %d ACC MODULE started\n",  __func__, __LINE__);
        bma250_set_state(SET_S);
        cfg_bma250_timers_start();
    }
    accelerometer_interrupt_init();

    while(1)
    {
        nrf_delay_ms(200);
        if(bma250_get_state() == NONE_ACC || bma250_get_state() == EXIT_ACC)
        {
            cfg_bma250_timers_stop();
            break;
        }
    }

//temperature sensor init
    tmp102_set_state(NONE_TMP);
    cfg_tmp102_timer_create();
    
    sd_app_evt_wait();
//temperature sensor run
    if(tmp102_get_state() == NONE_TMP)
    {

        cPrintLog(CDBG_MAIN_LOG, "%s %d TMP MODULE Started\n",  __func__, __LINE__);
        tmp102_set_state(TMP_SET_S);
        cfg_tmp102_timers_start();

//set threshold temperature bounds(upper and lower)
        threshold_low_result = tmp102_set_low_intr(TMP10x_INT_LOW);
        threshold_high_result = tmp102_set_high_intr(TMP10x_INT_HIGH);

        if((threshold_low_result == NRF_SUCCESS) && (threshold_high_result == NRF_SUCCESS))
            cPrintLog(CDBG_MAIN_LOG, "%s %d Temperature High/Low reg success \n",  __func__, __LINE__);
        else
            cPrintLog(CDBG_MAIN_LOG, "%s %d Temperature threshold reg fail \n",  __func__, __LINE__);

        while(1)
        {
            nrf_delay_ms(200);
            if(tmp102_get_state() == EXIT_TMP)
            {
// get temerature
                result = tmp102_get_tmp_data_once(&tmp102_int, &tmp102_dec);  /* Read the TMP10x sensor data value again. */
                if(result == TEC_SUCCESS)
                {   
                    cPrintLog(CDBG_MAIN_LOG, "Temperature TMP10x[%d.%d]'C \n", tmp102_int, tmp102_dec);
                }

                cfg_tmp102_timers_stop();
                tmp102_set_state(NONE_TMP);
                break;
            }
        }
    }
    cPrintLog(CDBG_MAIN_LOG, "%s %d TMP MODULE End\n",  __func__, __LINE__);
}
