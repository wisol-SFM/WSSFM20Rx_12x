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
 * @brief control temperature(tmp102) module.
 *
 * This file contains the control temperature(tmp102) module.
 */

#ifndef NRF_BLE_CFG_TMP102_H__
#define NRF_BLE_CFG_TMP102_H__

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
#include <stdint.h>
#include "cfg_bma250_module.h"

#define TMP102_SIZE 2
#define TMP102_SUM_S_CNT 2
#define TMP102_SUM_E_CNT 8
#define TMP102_AVERAGE_TOTAL_CNT (TMP102_SUM_E_CNT - TMP102_SUM_S_CNT)  // 6

#define TMP102_BITWISE_MATH 0.0625

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVER_NAME "tmp102"

/************************************************/
//#include <stdint.h>
/************************************************/


#define TMP102_CONF_REG_DATA_DEFAULT (0x60A0)
#define TMP108_CONF_REG_DATA_DEFAULT (0x2610)

#define TMP102_INIT_VALUE               ((uint8_t)0)
#define TMP102_GEN_READ_WRITE_LENGTH    ((uint8_t)1)
#define TMP102_LSB_MSB_READ_LENGTH      ((uint8_t)2)


#define TMP10X_I2C_ADDRESS 72 //TMP108_ADDR_GND// 72 /* This is the I2C address for our chip */

#define TMP102_TEMP_REG         0x00
#define TMP102_CONF_REG         0x01
#define TMP102_TLOW_REG         0x02
#define TMP102_THIGH_REG        0x03

/* interrupt set value : -30 ~ 80 'C */
#define TMP10x_INT_LOW           10 // 'c
#define TMP10x_INT_HIGH          40 // 'c
#define TMP10x_LOWHIGH_OFFSET    5 // 'c
#define TMP10x_DEFAULT_LOW_DATA  0x8000  // -128 'c
#define TMP10x_DEFAULT_HIGH_DATA 0x7FF8  // +127.9375 'c

/* note: these bit definitions are byte swapped */
#define TMP102_CONF_SD          0x0100
#define TMP102_CONF_TM          0x0200
#define TMP102_CONF_POL         0x0400
#define TMP102_CONF_F0          0x0800
#define TMP102_CONF_F1          0x1000
#define TMP102_CONF_R0          0x2000
#define TMP102_CONF_R1          0x4000
#define TMP102_CONF_OS          0x8000
#define TMP102_CONF_EM          0x0010
#define TMP102_CONF_AL          0x0020
#define TMP102_CONF_CR0         0x0040
#define TMP102_CONF_CR1         0x0080

#define TMP102_CONFREG_MASK (TMP102_CONF_SD | TMP102_CONF_TM | \
                 TMP102_CONF_POL | TMP102_CONF_F0 | \
                 TMP102_CONF_F1 | TMP102_CONF_OS | \
                 TMP102_CONF_EM | TMP102_CONF_AL | \
                 TMP102_CONF_CR0 | TMP102_CONF_CR1)

#define TMP102_CONFIG_CLEAR (TMP102_CONF_SD | TMP102_CONF_OS | \
                 TMP102_CONF_CR0)
#define TMP102_CONFIG_SET   (TMP102_CONF_TM | TMP102_CONF_EM | \
                 TMP102_CONF_CR1)

#define CONVERSION_TIME_MS      35  /* in milli-seconds */

struct tmp102 {
    struct regmap *regmap;
    uint16_t config_orig;
    unsigned long ready_time;
};

/* convert left adjusted 13-bit TMP102 register value to milliCelsius */
static inline int tmp102_reg_to_mC(int16_t val)
{
    return ((val & ~0x01) * 1000) / 128;
}

/* convert milliCelsius to left adjusted 13-bit TMP102 register value */
static inline uint16_t tmp102_mC_to_reg(int val)
{
    return (val * 128) / 1000;
}


/**@brief Structure for sigfox state machine. */
typedef enum
{
    NONE_TMP,
    TMP_SET_S,
    TMP_SET_R,
    TMP_READ_DATA_S,
    TMP_READ_DATA_R,
    TMP_READ_ONLY_S,
    TMP_READ_ONLY_R,
    TMP_SLEEP_S, //5
    TMP_SLEEP_R,
    TMP_INTERRUPT_S,
    TMP_INTERRUPT_R,
    TMP_INTERRUPT_CLEAR_S,
    TMP_INTERRUPT_CLEAR_R,
    EXIT_TMP
} tmp102_state_s;

typedef enum
{
    TMP_DISABLE,
    TMP_ENABLE,
    TMP_NONE,
} tmp102_sensor_s;

typedef enum
{
    TMP_INTR_NONE,
    TMP_INTR_LOW,
    TMP_INTR_HIGH,
} tmp102_event_intr_s;

typedef enum
{
    TMP_INTR_SET_DISABLE,  /* Interrupt low & high disable */
    TMP_INTR_SET_LOW,
    TMP_INTR_SET_HIGH,
    TMP_INTR_SET_LOWHIGH,
} tmp102_set_intr_s;

/* List of common tmp error codes that can be returned */
enum tec_error_list {
    /* Success - no error */
    TEC_SUCCESS = 0,
    /* Unknown error */
    TEC_ERROR_UNKNOWN = 1,
    /* Function not implemented yet */
    TEC_ERROR_UNIMPLEMENTED = 2,
    /* Overflow error; too much input provided. */
    TEC_ERROR_OVERFLOW = 3,
    /* Timeout */
    TEC_ERROR_TIMEOUT = 4,
    /* Invalid argument */
    TEC_ERROR_INVAL = 5,
    /* Already in use, or not ready yet */
    TEC_ERROR_BUSY = 6,
    /* Access denied */
    TEC_ERROR_ACCESS_DENIED = 7,
    TEC_ERROR_LAST =  0x1FFFF
};


/**
 * @brief       Function for TMP102 timer create
 */
void cfg_tmp102_timer_create(void);

/**
 * @brief       Function for TMP102 timer start
 */
void cfg_tmp102_timers_start(void);

/**
 * @brief       Function for TMP102 timer stop
 */
void cfg_tmp102_timers_stop(void);

/**
 * @brief       Function for TMP102 get state
 */
tmp102_state_s tmp102_get_state(void);

/**
 * @brief       Function for TMP102 set state
 */
void tmp102_set_state(tmp102_state_s m_state);

/**
 * @brief       Function for initializing i2c of tmp102
 */
void cfg_tmp102_i2c_init(void);

/**
 * @brief       Function for request shutdown of tmp102
 */
uint32_t tmp102_req_shutdown_mode(void);


/**
 * @brief       Function for set interrupt low/high temperature bounds(upper and lower) of tmp102
 */
void tmp102_set_lowhigh_intr(void);

/**
 * @brief       Function for set tmp102 i2c init
 */
void tmp102_i2c_init(void);

/**
 * @brief       Function for TMP10x get configuration register
 */
uint32_t tmp102_get_config(void);


/**
 * @brief       Function for TMP10x get temperature('C)
 */
uint32_t tmp102_get_tmp_data(void);

/**
 * @brief       Function for TMP10x get temperature('C) - just once
 */
uint32_t tmp102_get_tmp_data_once(int16_t *tmp102a, int16_t *tmp102b);

uint32_t tmp102_get_tmp(void);

uint32_t tmp102_set_low_intr(uint16_t tmp_min);

uint32_t tmp102_set_high_intr(uint16_t tmp_max);

#ifdef __cplusplus
}
#endif

#endif // NRF_BLE_CFG_MAIN_H__

/** @} */


