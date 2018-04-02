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
 * @brief tracking Sample Application cfg_tmp102_module.c file.
 *
 * This file contains the source code for an tmp102 sensor.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_log.h"
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
#include "cfg_dbg_log.h"
#include "cfg_tmp102_module.h"
#include "cfg_board_def.h"
#include "cfg_board.h"
#include "nrf_delay.h"

#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2) || (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
#define FEATURE_TMP108
#endif

extern volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/* TWI instance. */
extern const nrf_drv_twi_t m_twi;

//static uint8_t       m_tx_buf[20];            /**< TX buffer. */
static uint8_t       m_rx_buf[20];              /**< RX buffer. */
//static const uint8_t m_length;                /**< Transfer length. */


tmp102_state_s m_tmp102_state;

#define I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5
#define TMP102_BUS_READ_WRITE_ARRAY_INDEX 1
#define MPU_TWI_TIMEOUT     100000
int16_t tmp102a, tmp102b;
uint8_t tmp102a_sigfox, tmp102b_sigfox;
uint16_t tmp108_data;
extern volatile int main_wakeup_reason;
extern bool ble_connect_on;

APP_TIMER_DEF(m_tmp102_timer_id);                        /** TMP10x timer. */

#define CPRINTLOG_TMP false // register value log(default false)

void tmp102_i2c_init(void)
{
    cfg_i2c_master_uninit();
    cfg_i2c_master_init();
    nrf_delay_ms(10);
}

void tmp102_data_init(void)
{
    tmp102a = 0;
    tmp102b = 0;
    tmp102a_sigfox = 0;
    tmp102b_sigfox = 0;
}

tmp102_state_s tmp102_get_state()
{
    return m_tmp102_state;
}

void tmp102_set_state(tmp102_state_s m_state)
{
    m_tmp102_state = m_state;
}

void tmp102_tmp2data(uint16_t *data, int16_t temp)
{
    uint16_t con;
    
    if(temp < -30) temp =  -30;
    else if (temp > 80) temp = 80;
        
    if(temp >= 0)
    {
        //50/0.0625
        con = temp*16;
    }
    else
    {
        con = ~(abs(temp)*16) +1;
    }

    data[0] = (con & 0x0ff0) >> 4;
    data[1] = (con & 0x000f) << 4;
}

uint16_t tmp102_data2tmp(uint8_t *data, uint8_t *tmpa, uint8_t *tmpb)
{
    uint16_t ret_tmp = 0;
    int16_t tmp=0, tmp_a=0, tmp_b=0;

    if(data[0] & 0x80)
    {       // -65~-1
        tmp = (data[0] & 0xff) << 4;
        tmp |= data[1] >> 4;
        tmp |= 0x0f << 12;
    }
    else
    {
        tmp = (data[0] & 0xff) << 4;
        tmp |= data[1] >> 4;
    }
    tmp_a = (int16_t)(tmp / 16);
    tmp_b = (int16_t)(((tmp % 16) * 100) / 16);

/* tmp min & max : -40 ~ 125 'C */
    if(tmp_a > 125)
    {
        tmp_a = 125;
        tmp_b = 0;
    }
    else if(tmp_a < -40)
    {
        tmp_a = -40;
        tmp_b = 0;
    }
    tmp102a = tmp_a;
    tmp102b = tmp_b;

    if((tmp102a < 0) || (tmp102b < 0)) // -40 ~ -0.x
    {
        if(tmpa)*tmpa = ((tmp_a * -1) | 0x80);
        if(tmpb)*tmpb = (tmp_b * -1) | 0x80;
        ret_tmp = tmp108_data = (((tmp_a * -1) | 0x80) << 8) | ((tmp_b * -1) | 0x80);
    }
    else
    {
        
        if(tmpa)*tmpa = tmp_a;
        if(tmpb)*tmpb = tmp_b;
        ret_tmp = tmp108_data = (tmp_a << 8) | tmp_b;
    }
    return ret_tmp;
}

uint16_t tmp102_data2tmp_intr(uint8_t *data, int8_t *tmpa)
{
    uint16_t ret_tmp = 0;
    uint16_t tmp=0;
    int16_t tmp_a=0;

    if(data[0] & 0x80)
    {       // -65~-1
        tmp = (data[0] & 0xff) << 4;
        tmp |= data[1] >> 4;
        tmp |= 0x0f << 12;
    }
    else
    {
        tmp = (data[0] & 0xff) << 4;
        tmp |= data[1] >> 4;
    }
    tmp_a = (int16_t)(tmp / 16);
    if(tmpa)*tmpa = (int8_t)tmp_a;
    return ret_tmp;
}

/** 
*  Read register from TMP10x sensor. 
*/ 
uint8_t tmp102_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    uint32_t timeout = MPU_TWI_TIMEOUT;
    uint8_t stringpos = 0;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, TMP10X_I2C_ADDRESS,&reg_addr, 1, false));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, TMP10X_I2C_ADDRESS, reg_data, cnt));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    for (stringpos = 0; stringpos < cnt; stringpos++)
    {
        *(reg_data + stringpos) = reg_data[stringpos];
    }
    
    return NRF_SUCCESS;
}
  
/** 
*  Write register to TMP10x sensor. 
*/ 
uint8_t tmp102_i2c_write(uint8_t reg_addr, uint16_t reg_data/*, uint8_t cnt*/)
{
    uint32_t timeout = MPU_TWI_TIMEOUT;
    uint8_t array[4];

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    array[0] = reg_addr;
    array[1] =((reg_data & 0xff00) >> 8);  //byte 1
    array[2] =(uint8_t)(reg_data & 0xff);  //byte 2

    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, TMP10X_I2C_ADDRESS, array, 3, false));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    
    return NRF_SUCCESS;
}

void tmp102_intr_lowhigh_clear_state(void) 
{
    tmp102_set_state(TMP_INTERRUPT_CLEAR_S);
    cfg_tmp102_timers_stop();
    cfg_tmp102_timers_start();
}

uint32_t tmp102_req_shutdown_mode(void)
{
    uint16_t reg_data;
#ifdef FEATURE_TMP108
    reg_data = TMP108_CONF_REG_DATA_DEFAULT & (~(0x0300));  //clear M1 M0
#else
    reg_data = TMP102_CONF_REG_DATA_DEFAULT | TMP102_CONF_SD;   //SD(shutdown Mode) set 
#endif
    return tmp102_i2c_write(TMP102_CONF_REG,reg_data);
}

uint32_t tmp102_get_config()
{
    uint32_t result;

    result = tmp102_i2c_read(TMP102_CONF_REG,m_rx_buf,TMP102_LSB_MSB_READ_LENGTH);
    if(result != NRF_SUCCESS)return result;

    return result;
}

uint32_t tmp102_set_config()
{
    uint16_t reg_data;
    uint32_t result;

#ifdef FEATURE_TMP108
    reg_data = TMP108_CONF_REG_DATA_DEFAULT;
#else
    reg_data = TMP102_CONF_REG_DATA_DEFAULT;
#endif
    result = tmp102_i2c_write(TMP102_CONF_REG,reg_data);
    if(result != NRF_SUCCESS)return result;

    return result;
}

uint32_t tmp102_config_interrupt_test(void)
{
    uint16_t reg_data;
    uint32_t result;

#ifdef FEATURE_TMP108
    reg_data = (TMP108_CONF_REG_DATA_DEFAULT | 0x0080);  //POL to 1
#else
    reg_data = (TMP102_CONF_REG_DATA_DEFAULT | 0x0400);  //POL to 1
#endif
    result = tmp102_i2c_write(TMP102_CONF_REG,reg_data);
    if(result != NRF_SUCCESS)return result;

    return result;
}

uint32_t tmp102_get_low_intr(int8_t *low_intr )
{
    static uint32_t error_code = TEC_ERROR_UNKNOWN;
    int8_t tmp_intr;
    
    memset(m_rx_buf,0x00,sizeof(m_rx_buf));

    error_code = tmp102_i2c_read(TMP102_TLOW_REG,m_rx_buf, TMP102_LSB_MSB_READ_LENGTH); //TMP102_SHIFT_SIX_BITS);
    tmp102_data2tmp_intr(m_rx_buf,&tmp_intr);
    if(low_intr)*low_intr = tmp_intr;
    return error_code;
}

uint32_t tmp102_set_low_intr(uint16_t tmp_min)
{
    uint32_t result;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    spi_xfer_done = false;
    result = tmp102_i2c_write(TMP102_TLOW_REG,tmp_min);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    if(result != NRF_SUCCESS)return result;

    return result;
}


uint32_t tmp102_get_high_intr(int8_t *high_intr )
{
    static uint32_t error_code = TEC_ERROR_UNKNOWN;
    int8_t tmp_intr;

    memset(m_rx_buf,0x00,sizeof(m_rx_buf));

    error_code = tmp102_i2c_read(TMP102_THIGH_REG ,m_rx_buf, TMP102_LSB_MSB_READ_LENGTH);
    tmp102_data2tmp_intr(m_rx_buf,&tmp_intr);
    if(high_intr)*high_intr = tmp_intr;
    return error_code;
}

uint32_t tmp102_set_high_intr(uint16_t tmp_max)
{
    uint32_t result;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    spi_xfer_done = false;
    result = tmp102_i2c_write(TMP102_THIGH_REG,tmp_max);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    if(result != NRF_SUCCESS)return result;

    return result;
}

uint32_t tmp102_get_tmp_data(void)
{
    static uint32_t error_code = TEC_ERROR_UNKNOWN;
#if 0
	int32_t test_tmp;
    int16_t tmp102;  
#endif
	
    memset(m_rx_buf,0x00,sizeof(m_rx_buf));

    error_code = tmp102_i2c_read(TMP102_TEMP_REG,m_rx_buf, TMP102_LSB_MSB_READ_LENGTH);
    if(error_code == NRF_SUCCESS)
    {
#if 0
        if(m_rx_buf[0] & 0x80)
        {       // -65~-1
            tmp102 = (m_rx_buf[0] & 0xff) << 4;
            tmp102 |= m_rx_buf[1] >> 4;
            tmp102 |= 0x0f << 12;
        }else
        {
            tmp102 = (m_rx_buf[0] & 0xff) << 4;
            tmp102 |= m_rx_buf[1] >> 4;
        }

        test_tmp = (tmp102 * 625);
        test_tmp = test_tmp / 100;
        tmp102a = (test_tmp / 100);
        tmp102b = (test_tmp % 100);
#endif
        error_code = TEC_SUCCESS;
    }
    else
    {
        error_code = TEC_ERROR_ACCESS_DENIED;
    }
    return error_code;
}


uint32_t tmp102_get_tmp_data_once(int16_t *tmp102a, int16_t *tmp102b)
{
    static uint32_t error_code = TEC_ERROR_UNKNOWN;
    int32_t test_tmp;
    int16_t tmp102;  
    
    memset(m_rx_buf,0x00,sizeof(m_rx_buf));

    error_code = tmp102_i2c_read(TMP102_TEMP_REG,m_rx_buf, TMP102_LSB_MSB_READ_LENGTH);
    if(error_code == TEC_SUCCESS)
    {
        if(m_rx_buf[0] & 0x80)
        {       // -65~-1
            tmp102 = (m_rx_buf[0] & 0xff) << 4;
            tmp102 |= m_rx_buf[1] >> 4;
            tmp102 |= 0x0f << 12;
        }else
        {
            tmp102 = (m_rx_buf[0] & 0xff) << 4;
            tmp102 |= m_rx_buf[1] >> 4;
        }
        test_tmp = (tmp102 * 625);
        test_tmp = test_tmp / 100;
        if(tmp102a)*tmp102a = (test_tmp / 100);
        if(tmp102b)*tmp102b = (test_tmp % 100);
    }
    return error_code;
}

uint32_t tmp102_get_tmp(void)
{
    int16_t tmp_a, tmp_b;
    static uint32_t error_code = TEC_ERROR_UNKNOWN;

    error_code = tmp102_get_tmp_data_once(&tmp_a, &tmp_b);
    if(error_code == TEC_SUCCESS)
    {
        tmp102a = tmp_a;
        tmp102b = tmp_b;

        if((tmp102a < 0) || (tmp102b < 0)) // -40 ~ -0.x
            cPrintLog(CDBG_EXT_SEN_INFO, "%d Tmp10x sensor temperature:[-%d.%d] \n", __LINE__, tmp102a * (-1), tmp102b * (-1));
        else
            cPrintLog(CDBG_EXT_SEN_INFO, "%d Tmp10x sensor temperature:[%d.%d] \n", __LINE__, tmp102a, tmp102b);
    }
    else
    {
        error_code = TEC_ERROR_ACCESS_DENIED;
    }
    return error_code;
}

void tmp102_set_lowhigh_intr(void)
{
    uint16_t lowdata[2] = {0,};
    uint16_t highdata[2] = {0,};
    uint16_t tmp_low_reg, tmp_high_reg;
   
    tmp102_tmp2data(lowdata, TMP10x_DEFAULT_LOW_DATA);
    tmp_low_reg = (lowdata[0] << 8) | lowdata[1];

    tmp102_set_low_intr(tmp_low_reg);
    nrf_delay_ms(30);

    tmp102_tmp2data(highdata, TMP10x_DEFAULT_HIGH_DATA);
    tmp_high_reg = (highdata[0] << 8) | highdata[1];

    tmp102_set_high_intr(tmp_high_reg);
}

static void tmp102_state_handler(void * p_context)
{
    static uint32_t error_code;
    uint8_t tmpa, tmpb;
    int8_t intr_tmp_low, intr_tmp_high;
    
    switch(m_tmp102_state)
    {
        case TMP_SET_S:
            tmp102_i2c_init();
            tmp102_data_init();
            spi_xfer_done = false;

            error_code = tmp102_set_config();
            if(error_code != NRF_SUCCESS)
            {
                cPrintLog(CDBG_EXT_SEN_ERR, "%d TMP Sensor Init Error:%d\n", __LINE__, error_code);
                m_tmp102_state = EXIT_TMP;            
            }
            else
            {
                m_tmp102_state = TMP_SET_R;
            }
            break;

        case TMP_SET_R:
            tmp102_get_low_intr(&intr_tmp_low);
            tmp102_get_high_intr(&intr_tmp_high);
            m_tmp102_state = TMP_INTERRUPT_S;
            break;
            
        case TMP_INTERRUPT_S:
            tmp102_set_lowhigh_intr();
            m_tmp102_state = TMP_INTERRUPT_R;
            break;

        case TMP_INTERRUPT_R:
            m_tmp102_state = TMP_READ_DATA_S;
            break;

        case TMP_READ_DATA_S:
            spi_xfer_done = false;
            error_code = tmp102_get_tmp_data();

            if(error_code != NRF_SUCCESS)
            {
                cPrintLog(CDBG_EXT_SEN_ERR, "%d TMP Sensor Init Error:%d\n", __LINE__, error_code);
                nrf_delay_ms(10);
                m_tmp102_state = EXIT_TMP;            
            }
            else
            {
                m_tmp102_state = TMP_READ_DATA_R;
            }
        
            break;
        
        case TMP_READ_DATA_R:
            tmp102_data2tmp(m_rx_buf, &tmpa, &tmpb);
            tmp102a_sigfox = tmpa;
            tmp102b_sigfox = tmpb;
            
#ifdef CDEV_NUS_MODULE
            m_nus_service_parameter.temperature_data[0] = tmp102a_sigfox;
            m_nus_service_parameter.temperature_data[1] = tmp102b_sigfox;
#endif
            if((tmp102a < 0) || (tmp102b < 0)) // -40 ~ -0.x
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "%d Tmp10x sensor temperature:[-%d.%d] \n", __LINE__, tmp102a * (-1), tmp102b * (-1));
                // temp cPrintLog(CDBG_EXT_SEN_INFO, "tmp102a_sigfox[0x%02x][%d] tmp102b_sigfox[0x%02x][%d]\n", tmp102a_sigfox, tmp102a_sigfox, tmp102b_sigfox, tmp102b_sigfox);
            }
            else
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "%d Tmp10x sensor temperature:[%d.%d] \n", __LINE__, tmp102a, tmp102b);
            }
            m_tmp102_state = EXIT_TMP;
            spi_xfer_done = false;
            break;
            
        case TMP_READ_ONLY_S:
            spi_xfer_done = false;
            tmp102_get_tmp_data();
            tmp102_data2tmp(m_rx_buf, &tmpa, &tmpb);
            tmp102a_sigfox = tmpa;
            tmp102b_sigfox = tmpb;
            if((tmp102a < 0) || (tmp102b < 0)) // -40 ~ -0.x
                cPrintLog(CDBG_EXT_SEN_INFO, "%d Tmp10x sensor temperature:[-%d.%d] \n", __LINE__, tmp102a * (-1), tmp102b * (-1));
            else
                cPrintLog(CDBG_EXT_SEN_INFO, "%d Tmp10x sensor temperature:[%d.%d] \n", __LINE__, tmp102a, tmp102b);
            cfg_tmp102_timers_stop();

            m_tmp102_state = NONE_TMP;            
            break;
            
        case TMP_READ_ONLY_R:
            m_tmp102_state = TMP_READ_ONLY_S;
            break;

        case TMP_SLEEP_S:
            m_tmp102_state = TMP_SLEEP_R;
            break;

        case TMP_SLEEP_R:
            m_tmp102_state = EXIT_TMP;
            break;
            
        case TMP_INTERRUPT_CLEAR_S:
            break;
            
        case TMP_INTERRUPT_CLEAR_R:
            break;

        case EXIT_TMP:
            break;

        default:
            break;
    }
}

void cfg_tmp102_timer_create()
{
    uint32_t err_code;

    // Create timers.
    err_code = app_timer_create(&m_tmp102_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                tmp102_state_handler);
    APP_ERROR_CHECK(err_code);
}

void cfg_tmp102_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_tmp102_timer_id, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}
void cfg_tmp102_timers_stop(void)
{
    uint32_t err_code;

    // Stop application timers.
    err_code = app_timer_stop(m_tmp102_timer_id);
    APP_ERROR_CHECK(err_code);
}

