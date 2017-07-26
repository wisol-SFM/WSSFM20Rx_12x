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
 * @brief tracking Sample Application cfg_external_sense_gpio.c file.
 *
 * This file contains the source code for an tracking sample application.
 */

#include <stdint.h>
#include <string.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_gpio.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#include "cfg_dbg_log.h"
#include "cfg_board_def.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_twis_board_control.h"
#include "cfg_board.h"
#include "cfg_external_sense_gpio.h"

static bool m_magnetic_old_status;
magnetic_attach_callback m_magnetic_attach_CB = NULL;
wkup_detect_callback m_wkup_detect_CB = NULL;
    

APP_TIMER_DEF(m_magnetic_timer_id);
static bool cfg_magnetic_status_set(bool status)
{
    bool updated = false;
    if(status != m_magnetic_old_status)
    {
        if(status)
        {
            cPrintLog(CDBG_EXT_SEN_INFO, "%s The magnet is attached!\n", __func__);
            cTBC_write_state_noti("MagnetAttached");
            if(m_magnetic_attach_CB)m_magnetic_attach_CB();
#ifdef CDEV_NUS_MODULE
            m_nus_service_parameter.magnet_event = '1';
#endif     
            m_module_peripheral_data.magnet_status = 1;
        }
        else
        {
            cPrintLog(CDBG_EXT_SEN_INFO, "%s The magnet is detached!\n", __func__);
            cTBC_write_state_noti("MagnetDetached");
            m_module_peripheral_data.magnet_status = 0;
        }
        m_magnetic_old_status = status;
        updated = true;
    }
    return updated;
}
static void cfg_magnetic_timeout_handler(void * p_context)
{
    bool magnetic_status;
    nrf_drv_gpiote_in_event_enable(PIN_DEF_MAGNETIC_SIGNAL, true);
    magnetic_status = !nrf_drv_gpiote_in_is_set(PIN_DEF_MAGNETIC_SIGNAL);
    cfg_magnetic_status_set(magnetic_status);
}

static void cfg_magnetic_isr_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    bool magnetic_status;
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_TOGGLE:
            if(pin == PIN_DEF_MAGNETIC_SIGNAL)
            {
                magnetic_status = !nrf_drv_gpiote_in_is_set(PIN_DEF_MAGNETIC_SIGNAL);
                if(cfg_magnetic_status_set(magnetic_status))  //for interrupt filter
                {
                    app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_ISR_IGNORE_TIME_MS, APP_TIMER_PRESCALER), NULL);
                    nrf_drv_gpiote_in_event_disable(PIN_DEF_MAGNETIC_SIGNAL);
                }
            }
            break;

        default:
            cPrintLog(CDBG_EXT_SEN_ERR, "%s bad pin:%d, action:%d\n", __func__, pin, action);
            break;
    }
}

/**
 * @brief Function for initializing the Magnetic Sensor. (attached low signal)
 * @param[in]   callback callback function for magnetic attacked
 */
void cfg_magnetic_sensor_init(magnetic_attach_callback callback)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    config.pull = NRF_GPIO_PIN_NOPULL;  //pull up or pull down or no pull
    err_code = nrf_drv_gpiote_in_init(PIN_DEF_MAGNETIC_SIGNAL, &config, cfg_magnetic_isr_event_handler);
    if(err_code==NRF_SUCCESS)
    {
        nrf_drv_gpiote_in_event_enable(PIN_DEF_MAGNETIC_SIGNAL, true);
        cPrintLog(CDBG_EXT_SEN_INFO, "%s magnetic sensor started!\n", __func__);
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "%s nrf_drv_gpiote_in_event_enable error:%d\n", __func__,err_code);
    }
    m_magnetic_old_status = !nrf_drv_gpiote_in_is_set(PIN_DEF_MAGNETIC_SIGNAL);
    err_code = app_timer_create(&m_magnetic_timer_id, APP_TIMER_MODE_SINGLE_SHOT, cfg_magnetic_timeout_handler);
    APP_ERROR_CHECK(err_code);

    m_magnetic_attach_CB = callback;
}

static void cfg_wkup_gpio_isr_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_LOTOHI:
            if(pin == PIN_DEF_WKUP)
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "WKUP Signal Detected!\n");
                if(m_wkup_detect_CB)m_wkup_detect_CB();
            }
            break;

        default:
            cPrintLog(CDBG_EXT_SEN_ERR, "%s bad pin:%d, action:%d\n", __func__, pin, action);
            break;
    }
}

/**
 * @brief Function for initializing the wake up gpio. (active low signal)
 * @param[in]   callback callback function for wkup key push (active high)
 */
void cfg_wkup_gpio_init(wkup_detect_callback callback)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t config_wkup = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    config_wkup.pull = NRF_GPIO_PIN_PULLDOWN;  //pull up or pull down or no pull
    err_code = nrf_drv_gpiote_in_init(PIN_DEF_WKUP, &config_wkup, cfg_wkup_gpio_isr_event_handler);
    if(err_code==NRF_SUCCESS)
    {
        nrf_drv_gpiote_in_event_enable(PIN_DEF_WKUP, true);
        cPrintLog(CDBG_EXT_SEN_INFO, "%s wkup sense started\n", __func__, err_code);
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "%s nrf_drv_gpiote_in_event_enable error:%d\n", __func__,err_code);
    }

    m_wkup_detect_CB = callback;
}
