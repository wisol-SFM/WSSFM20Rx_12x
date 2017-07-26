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


#include "mma7660.h"


uint8_t const mma7660_xout_reg_addr = MMA7660_REG_XOUT;


// Set Active mode.
static uint8_t const default_config[] = { MMA7660_REG_MODE, 1 };
app_twi_transfer_t const mma7660_init_transfers[MMA7660_INIT_TRANSFER_COUNT] =
{
    APP_TWI_WRITE(MMA7660_ADDR, default_config, sizeof(default_config), 0)
};
