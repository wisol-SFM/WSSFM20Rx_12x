/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */


#include "lm75b.h"


uint8_t const lm75b_conf_reg_addr  = LM75B_REG_CONF;
uint8_t const lm75b_temp_reg_addr  = LM75B_REG_TEMP;
uint8_t const lm75b_tos_reg_addr   = LM75B_REG_TOS;
uint8_t const lm75b_thyst_reg_addr = LM75B_REG_THYST;


// Set default configuration of LM75B - write 0 to Conf register.
static uint8_t const default_config[] = { LM75B_REG_CONF, 0 };
app_twi_transfer_t const lm75b_init_transfers[LM75B_INIT_TRANSFER_COUNT] =
{
    APP_TWI_WRITE(LM75B_ADDR, default_config, sizeof(default_config), 0)
};
