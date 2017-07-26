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

#ifndef LM75B_H__
#define LM75B_H__


#include "app_twi.h"

#ifdef __cplusplus
extern "C" {
#endif


// 0x90 is the LM75B's address in the mbed Application Shield, it contains
// R/W bit and "nrf_drv_twi" (and consequently "app_twi") requires slave
// address without this bit, hence shifting.
#define LM75B_ADDR          (0x90U >> 1)

#define LM75B_REG_TEMP      0x00
#define LM75B_REG_CONF      0x01
#define LM75B_REG_THYST     0x02
#define LM75B_REG_TOS       0x03

// [use "/ 32" instead of ">> 5", as the result of right-shifting of a signed
//  type value is implementation-defined]
#define LM75B_GET_TEMPERATURE_VALUE(temp_hi, temp_lo) \
    ((((int16_t)temp_hi << 8) | temp_lo) / 32)


extern uint8_t const lm75b_conf_reg_addr;
extern uint8_t const lm75b_temp_reg_addr;
extern uint8_t const lm75b_tos_reg_addr;
extern uint8_t const lm75b_thyst_reg_addr;


#define LM75B_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(LM75B_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (LM75B_ADDR, p_buffer,   byte_cnt, 0)

#define LM75B_READ_TEMP(p_buffer) \
    LM75B_READ(&lm75b_temp_reg_addr, p_buffer, 2)

#define LM75B_INIT_TRANSFER_COUNT 1
extern app_twi_transfer_t const lm75b_init_transfers[LM75B_INIT_TRANSFER_COUNT];



#ifdef __cplusplus
}
#endif

#endif // LM75B_H__
