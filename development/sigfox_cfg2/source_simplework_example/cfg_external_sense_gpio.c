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
static uint32_t m_magnetic_detect_tick;
static magnetic_attach_callback m_magnetic_attach_CB = NULL;
static magnetic_detach_callback m_magnetic_detach_CB = NULL;
static wkup_detect_callback m_wkup_detect_CB = NULL;

static nrf_drv_gpiote_pin_t m_button_pin;

static int m_button_press_tick;
static bool m_button_sense_started = false;
static bool m_is_active_high;

static wkup_detect_callback m_button_short_press_CB = NULL;
static wkup_detect_callback m_button_long_press_CB = NULL;

APP_TIMER_DEF(m_magnetic_timer_id);
APP_TIMER_DEF(m_button_id);

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
            if(m_magnetic_detach_CB)m_magnetic_detach_CB();
            m_module_peripheral_data.magnet_status = 0;
        }
        m_magnetic_old_status = status;
        updated = true;
    }
    return updated;
}
#define cfg_magnetic_get_status() !nrf_drv_gpiote_in_is_set(PIN_DEF_MAGNETIC_SIGNAL);
static void cfg_magnetic_timeout_handler(void * p_context)
{
    bool magnetic_status;
    nrf_drv_gpiote_in_event_enable(PIN_DEF_MAGNETIC_SIGNAL, true);
    magnetic_status = cfg_magnetic_get_status();
    if(m_magnetic_old_status != magnetic_status)
    {
        if(++m_magnetic_detect_tick > GFG_MAGNETIC_DETECT_IGNORE_TICK_CNT)
        {
            nrf_drv_gpiote_in_event_enable(PIN_DEF_MAGNETIC_SIGNAL, true);
            magnetic_status = cfg_magnetic_get_status();
            cfg_magnetic_status_set(magnetic_status);
        }
        else
        {
            app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_DETECT_TICK_TIME_MS, APP_TIMER_PRESCALER), NULL);
        }
    }
    else
    {
        nrf_drv_gpiote_in_event_enable(PIN_DEF_MAGNETIC_SIGNAL, true);
    }
}

static void cfg_magnetic_isr_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    bool magnetic_status;
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_TOGGLE:
            if(pin == PIN_DEF_MAGNETIC_SIGNAL)
            {
                magnetic_status = cfg_magnetic_get_status();
                if(m_magnetic_old_status != magnetic_status)
                {
                    //polling start
                    m_magnetic_detect_tick = 0;
                    nrf_drv_gpiote_in_event_disable(PIN_DEF_MAGNETIC_SIGNAL);
                    app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_DETECT_TICK_TIME_MS, APP_TIMER_PRESCALER), NULL);
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
void cfg_magnetic_sensor_init(magnetic_attach_callback callback_attach, magnetic_detach_callback callback_detach)
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
    m_magnetic_old_status = cfg_magnetic_get_status();
    err_code = app_timer_create(&m_magnetic_timer_id, APP_TIMER_MODE_SINGLE_SHOT, cfg_magnetic_timeout_handler);
    APP_ERROR_CHECK(err_code);

    m_magnetic_attach_CB = callback_attach;
    m_magnetic_detach_CB = callback_detach;
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

static void cfg_button_timer_start(void)
{
    app_timer_start(m_button_id, APP_TIMER_TICKS(GFG_BUTTON_SENSE_TICKE_MS, APP_TIMER_PRESCALER), NULL);
}

static void cfg_button_sense_start(void)
{
    cPrintLog(CDBG_EXT_SEN_INFO, "button sense started, io:%d, lvl:%d\n", m_button_pin, m_is_active_high);
    m_button_sense_started = true;
    m_button_press_tick = 0;
    nrf_drv_gpiote_in_event_enable(m_button_pin, true);
}

static void cfg_button_timeout_handler(void * p_context)
{
    bool io_status;
    m_button_press_tick++;
    io_status = nrf_drv_gpiote_in_is_set(m_button_pin);

    if(!m_button_sense_started)
    {
        if(io_status == m_is_active_high)
        {
            cfg_button_timer_start();
        }
        else
        {
            cfg_button_sense_start();
        }
    }
    else
    {

        if(io_status == m_is_active_high)
        {
            //press key
            if(m_button_press_tick == GFG_BUTTON_SENSE_PRESS_TICK_LONG)
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "BUTTON press long!\n");
                if(m_button_long_press_CB)m_button_long_press_CB();
            }
            cfg_button_timer_start();
        }
        else
        {
            //release key
            if(m_button_press_tick > GFG_BUTTON_SENSE_PRESS_TICK_SHORT && m_button_press_tick < GFG_BUTTON_SENSE_PRESS_TICK_LONG)
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "BUTTON press short!\n");
                if(m_button_short_press_CB)m_button_short_press_CB();
            }
            m_button_press_tick = 0;
            nrf_drv_gpiote_in_event_enable(m_button_pin, true);
        }
    }
}

static void cfg_button_isr_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_LOTOHI:
        case NRF_GPIOTE_POLARITY_HITOLO:
            if(m_is_active_high)
            {
                if(action != NRF_GPIOTE_POLARITY_LOTOHI)
                {
                    cPrintLog(CDBG_EXT_SEN_ERR, "bad button isr H\n");
                }
            }
            else
            {
                if(action != NRF_GPIOTE_POLARITY_HITOLO)
                {
                    cPrintLog(CDBG_EXT_SEN_ERR, "bad button isr L\n");
                }
            }
            nrf_drv_gpiote_in_event_disable(m_button_pin);
            cfg_button_timer_start();
            break;

        default:
            cPrintLog(CDBG_EXT_SEN_ERR, "bad button isr pin:%d, action:%d\n", pin, action);
            break;
    }
}

void cfg_button_init(bool is_active_high, uint32_t pin, usr_def_button_short_press_callback short_press_CB, usr_def_button_long_press_callback long_press_CB)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t config_h = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    nrf_drv_gpiote_in_config_t config_l = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    nrf_drv_gpiote_in_config_t *config_ptr;
    bool io_status;

    m_button_pin = pin;
    m_button_short_press_CB = short_press_CB;
    m_button_long_press_CB = long_press_CB;
    m_is_active_high = is_active_high;

    if(m_is_active_high)
    {
        config_h.pull = NRF_GPIO_PIN_PULLDOWN;  //pull up or pull down or no pull
        config_ptr = &config_h;
    }
    else
    {
        config_l.pull = NRF_GPIO_PIN_PULLUP;  //pull up or pull down or no pull
        config_ptr = &config_l;
    }

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_gpiote_in_init(m_button_pin, config_ptr, cfg_button_isr_event_handler);
    if(err_code==NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_INFO, "button sense init! pin:%d, active lvl:%d\n", m_button_pin, m_is_active_high);
        io_status = nrf_drv_gpiote_in_is_set(m_button_pin);
        err_code = app_timer_create(&m_button_id, APP_TIMER_MODE_SINGLE_SHOT, cfg_button_timeout_handler);
        APP_ERROR_CHECK(err_code);
        if(io_status == m_is_active_high)
        {
            m_button_sense_started = false;
            cfg_button_timer_start();
        }
        else
        {
            cfg_button_sense_start();
        }
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "%s nrf_drv_gpiote_in_event_enable error:%d\n", __func__, err_code);
    }
}
void cfg_button_release(void)
{
    nrf_drv_gpiote_in_event_disable(m_button_pin);
    nrf_drv_gpiote_in_uninit(m_button_pin);
    m_button_sense_started = false;
}


