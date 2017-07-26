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
 * This file contains the source code for an tracking sample application.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_drv_spi.h"
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

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(GSEN_SPI_INSTANCE);  /**< SPI instance. */
extern volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/* TWI instance. */
extern const nrf_drv_twi_t m_twi;
unsigned short tmp_average;

//static uint8_t       m_tx_buf[20];           /**< TX buffer. */
static uint8_t       m_rx_buf[20];    /**< RX buffer. */
//static const uint8_t m_length;        /**< Transfer length. */

//static struct bma2x2_t *p_bma2x2;
//struct bma2x2_accel_data m_accel;

tmp102_state_s m_tmp102_state;
//tmp102_cmd_c m_tmp102_cmd;

#define I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5
#define TMP102_BUS_READ_WRITE_ARRAY_INDEX 1
//#define TMP102_SPI_BUS_WRITE_CONTROL_BYTE 0x7F
//#define TMP102_SPI_BUS_READ_CONTROL_BYTE 0x80
#define MPU_TWI_TIMEOUT     100000
static int cnt;
//#define         TMP102_INIT_VALUE                       ((u16)0)
int tmp102a, tmp102b;

APP_TIMER_DEF(m_tmp102_timer_id);                        /**BMA250 timer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_tmp102_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

void cfg_tmp102_spi_init(void)
{
    nrf_drv_spi_config_t spi_config;
    memcpy(&spi_config, &m_spi_config_default, sizeof(nrf_drv_spi_config_t));
    spi_config.ss_pin   = 20;
    spi_config.miso_pin = 16;
    spi_config.mosi_pin = 15;
    spi_config.sck_pin  = 18;
//    spi_config.frequency
//    spi_config.bit_order
//    spi_config.irq_priority
    spi_config.mode = NRF_DRV_SPI_MODE_3;
//    spi_config.orc
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_tmp102_handler));
}


tmp102_state_s tmp102_get_state()
{
    return m_tmp102_state;
}

void tmp102_set_state(tmp102_state_s m_state)
{
//    cPrintLog(CDBG_FCTRL_INFO, "%s  m_state[%d]  started\n",  __func__, m_tmp102_state);
    m_tmp102_state = m_state;
}

void tmp102_spi_bus_write(u8 reg_addr, u8 *reg_data, u8 cnt)
{

    u8 array[SPI_BUFFER_LEN * 2];
    u8 stringpos = TMP102_INIT_VALUE;

    for (stringpos = TMP102_INIT_VALUE; stringpos < cnt; stringpos++) {
        /* the operation of (reg_addr++)&0x7F done:
        because it ensure the
        0 and 1 of the given value
        It is done only for 8bit operation*/
        array[stringpos * 2] = (reg_addr++);// &
//        TMP102_SPI_BUS_WRITE_CONTROL_BYTE;
        array[stringpos * 2 + TMP102_BUS_READ_WRITE_ARRAY_INDEX] =
        *(reg_data + stringpos);
    }
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, array, cnt*2, NULL, 0));
}

void tmp102_SPI_bus_read(u8 reg_addr, u8 *reg_data, u8 cnt)
{
//    32 iError = TMP102_INIT_VALUE;
    u8 array[SPI_BUFFER_LEN] = {0xFF};
    u8 stringpos;
//    uint8_t count = 0;
/*  For the SPI mode only 7 bits of register addresses are used.
The MSB of register address is declared the bit what functionality it is
read/write (read as 1/write as 0)*/
array[TMP102_INIT_VALUE] = reg_addr/*|TMP102_SPI_BUS_READ_CONTROL_BYTE*/;
/*read routine is initiated register address is mask with 0x80*/
/*
* Please take the below function as your reference for
* read the data using SPI communication
* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
* add your SPI read function here
* iError is an return value of SPI read function
* Please select your valid return value
* In the driver SUCCESS defined as 0
* and FAILURE defined as -1
* Note :
* This is a full duplex operation,
* The first read data is discarded, for that extra write operation
* have to be initiated. For that cnt+1 operation done in the SPI read
* and write string function
* For more information please refer data sheet SPI communication:
*/
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, array,1 , reg_data, cnt+1));
    for (stringpos = TMP102_INIT_VALUE; stringpos < cnt; stringpos++) {
        *(reg_data + stringpos) = array[stringpos +
        TMP102_BUS_READ_WRITE_ARRAY_INDEX];

    }


}

uint32_t tmp102_i2c_reg_write(u8 reg_addr, u16 reg_data)  //not support burst mode reg_data size is 2byte
{
    uint32_t timeout = MPU_TWI_TIMEOUT;
    uint8_t array[4];

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    array[0] = reg_addr;
    array[1] = ((reg_data & 0xff00) >> 8);  //byte 1
    array[2] = (uint8_t)(reg_data & 0xff);  //byte 2
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, TMP102_I2C_ADDRESS, array, 3, false));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    return NRF_SUCCESS;
}


uint32_t tmp102_i2c_reg_read(u8 reg_addr, u8 *reg_data, u8 cnt)
{

    u8 array[I2C_BUFFER_LEN] = {TMP102_INIT_VALUE};
    u8 stringpos = TMP102_INIT_VALUE;
    uint32_t timeout = MPU_TWI_TIMEOUT;

//  array[TMP102_INIT_VALUE] = reg_addr;
/* Please take the below function as your reference
* for read the data using I2C communication
* add your I2C rad function here.
* "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
* iError is an return value of SPI write function
* Please select your valid return value
* In the driver SUCCESS defined as 0
* and FAILURE defined as -1
*/

//    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, TMP102_I2C_ADDR1,&reg_addr, 1, false));
//    while((!spi_xfer_done) && --timeout);
//    array[TMP102_INIT_VALUE] = 0x00;//reg_addr;


// TLOW Register (Read/Write)

    spi_xfer_done = false;
    array[0] = TMP102_TEMP_REG;
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, TMP102_I2C_ADDRESS,array, cnt, true));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi,TMP102_I2C_ADDRESS, reg_data, cnt+1));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    for (stringpos = TMP102_INIT_VALUE; stringpos < cnt; stringpos++)
        *(reg_data + stringpos) = reg_data[stringpos];
    return NRF_SUCCESS;

}

uint32_t tmp102_req_shutdown_mode(void)
{
    u16 reg_data;
    reg_data = TMP102_CONF_REG_DATA_DEFAULT | 0x0100;   //SD(shutdown Mode) set 
    return tmp102_i2c_reg_write(TMP102_CONF_REG,reg_data);
}

uint32_t tmp102_config_set()
{
    u16 reg_data;
    uint32_t result;

    reg_data = TMP102_CONF_REG_DATA_DEFAULT;
    result = tmp102_i2c_reg_write(TMP102_CONF_REG,reg_data);
    if(result != NRF_SUCCESS)return result;

    return result;
}

uint32_t tmp102_read_sensor()
{
    static uint32_t error_code = NRF_ERROR_NULL;
    memset(m_rx_buf,0x00,sizeof(m_rx_buf));

    error_code = tmp102_i2c_reg_read(TMP102_TEMP_REG,m_rx_buf, TMP102_LSB_MSB_READ_LENGTH); //TMP102_SHIFT_SIX_BITS);
    return error_code;
}

int tmp102_get_result(int *tmp102_int, int *tmp102_dec)
{
    int result = 0;

    *tmp102_int = tmp102a;
    *tmp102_dec = tmp102b;
    if((tmp102a != 0) || (tmp102b != 0))
    {
        result= true;
    }
    else
    {
        result = false;
    }
    
    return result;
}
int get_temperature(int *tmp_int, int *tmp_dec)
{
    int result = 0;
    result = tmp102_get_result(tmp_int, tmp_dec);

    return result;
}

uint32_t set_alert_threshold(u16 tmp_min, u16 tmp_max)
{
    uint32_t result;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    spi_xfer_done = false;
    result = tmp102_i2c_reg_write(TMP102_TLOW_REG,tmp_min);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    if(result != NRF_SUCCESS)return result;

    spi_xfer_done = false;
    result = tmp102_i2c_reg_write(TMP102_THIGH_REG,tmp_max);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    if(result != NRF_SUCCESS)return result;

    return result;
}

static void tmp102_state_handler(void * p_context)
{
    static uint32_t error_code;

    u8 firstbyte = {0};
    u8 secondbyte = {0};
    static unsigned short tmp_sum;
    static unsigned short tmp_dataU16;
    char    buffer[20];
    int tmp102;  
    uint32_t test_tmp;

    switch(m_tmp102_state)
    {
        case TMP_SET_S:
            cfg_i2c_master_uninit();
            cfg_i2c_master_init();
            nrf_delay_ms(1);
            cnt = 0;
            tmp_average = 0;
            tmp_sum = 0;
            spi_xfer_done = false;
            error_code = tmp102_config_set();
            if(error_code != NRF_SUCCESS)
                cPrintLog(CDBG_EXT_SEN_ERR, "%d TMP Sensor Init Error:%d\n", __LINE__, error_code);
            m_tmp102_state = TMP_SET_R;
            break;

        case TMP_SET_R:
            if(error_code == NRF_SUCCESS)
            {
                spi_xfer_done = false;
            }
            else
            {
            }
            m_tmp102_state = TMP_READ_DATA_S;
            break;

        case TMP_READ_DATA_S:
            spi_xfer_done = false;
            error_code = tmp102_read_sensor();

            m_tmp102_state = TMP_READ_DATA_R;
            break;
            case TMP_READ_DATA_R:
            tmp_dataU16 = 0;
            memcpy(&firstbyte,&m_rx_buf[0],1);
            memcpy(&secondbyte,&m_rx_buf[1],1);
            cnt++;

            if(spi_xfer_done)
            {
                tmp_dataU16 = ((firstbyte << 4) | ((secondbyte & 0xf0) >> 4));

                if ( tmp_dataU16 != 0 /*NULL*/ )
                {
                    if((cnt > TMP102_SUM_S_CNT) && (cnt <= TMP102_SUM_E_CNT))
                    {
                        tmp_sum = tmp_sum + tmp_dataU16;
                        m_tmp102_state = TMP_READ_DATA_S;
                    }
                    else if (cnt > TMP102_SUM_E_CNT)
                    {
                        cnt = 0;
                        tmp_average = tmp_sum / TMP102_AVERAGE_TOTAL_CNT;

                        sprintf( buffer, "%d", tmp_average );
                        tmp102 = atoi(buffer);
                        test_tmp = (tmp102 * 625);
                        test_tmp = test_tmp / 100;
                        tmp102a = test_tmp / 100;
                        tmp102b = test_tmp % 100;
#ifdef CDEV_NUS_MODULE
                        m_nus_service_parameter.temperature_data[0] = tmp102a;
                        m_nus_service_parameter.temperature_data[1] = tmp102b;
#endif
                        m_module_peripheral_data.temperature_data[0] = tmp102a;
                        m_module_peripheral_data.temperature_data[1] = tmp102b;
                        cPrintLog(CDBG_EXT_SEN_INFO, "%d TMP Avg Val a:%d, b:%d\n", __LINE__, tmp102a, tmp102b);
                        //                        if(1)cPrintLog(CDBG_EXT_SEN_INFO, "tmp102a[%02x] tmp102b[%02x]\n", tmp102a, tmp102b);
                        m_tmp102_state = EXIT_TMP;
                    }
                    else
                    {
                        m_tmp102_state = TMP_READ_DATA_S;
                    }
                }
                else
                {
                    if(cnt > 10)
                    {
                        cnt = 0;
                        tmp_average = 0;
                        m_tmp102_state = EXIT_TMP;
                    }
                    else
                        m_tmp102_state = TMP_READ_DATA_S;
                }
                spi_xfer_done = false;
            }
            else
            {
                if(cnt > 10)
                {
                    cnt = 0;
                    tmp_average = 0;
                    m_tmp102_state = EXIT_TMP;
                }
                else
                    m_tmp102_state = TMP_READ_DATA_S;
            }
            break;

        case TMP_SLEEP_S:
//            m_tmp102_state = TMP_SLEEP_R;
            break;


        case TMP_SLEEP_R:
            m_tmp102_state = EXIT_TMP;
            break;

        case EXIT_TMP:
            cfg_i2c_master_uninit();
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

    // Start application timers.
    err_code = app_timer_stop(m_tmp102_timer_id);
    APP_ERROR_CHECK(err_code);
}

