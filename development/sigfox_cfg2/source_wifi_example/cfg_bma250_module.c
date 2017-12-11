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
 * @brief tracking Sample Application cfg_bma250_module.c file.
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
#include "cfg_bma250_module.h"
#include "bma2x2.h"
#include "cfg_board_def.h"
#include "cfg_board.h"
#include "nrf_delay.h"

extern volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/* TWI instance. */
extern const nrf_drv_twi_t m_twi;


static uint8_t       m_tx_buf[20];           /**< TX buffer. */
static uint8_t       m_rx_buf[20];    /**< RX buffer. */

struct bma_accel_data m_accel;

bma250_state_s m_bma250_state;

#define I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5
#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX   1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE   0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE    0x80
#define MPU_TWI_TIMEOUT                     100000

#define COMMAND_SIZE 9

APP_TIMER_DEF(m_bma250_timer_id);                        /**BMA250 timer. */


#ifdef FEATURE_CFG_BYPASS_CONTROL

uint8_t input_control[10];

struct bma2x2_bypass_data {
    uint8_t mode;
    uint8_t reg;
    uint8_t value;
};

struct bma2x2_bypass_data m_bypass_data;

static uint8_t hextonum(uint8_t c)
{
    if( '0' <= c && c <= '9' )
        return c - '0';

    if( 'a' <= c && c <= 'f' )
        return c - 'a' + 10;

    if( 'A' <= c && c <= 'F' )
        return c - 'A' + 10;

    return 0;
}

void hexdigit_to_hexnum(uint8_t* dec, uint8_t* src, uint8_t size)
{
    uint8_t dec_start;
    uint8_t src_start;

    for(src_start=0,dec_start=0 ; src_start < size ;)
    {
        dec[dec_start] |=hextonum(src[src_start++])<<4;
        dec[dec_start++] |=hextonum(src[src_start++]); 
    }
}

bool acc_is_bypass_mode(void)
{
    if((m_bma250_state == BYPASS_S)||(m_bma250_state == BYPASS_R)||(m_bma250_state == READ_DATA_S)||(m_bma250_state == READ_DATA_R))
        return true;
    return false;
}

void acc_bypass_put(const char * data, int size)
{
    int i, j;
        
    memset(input_control,0,10);
    if(size != COMMAND_SIZE+2)
    {
        sprintf((char *)input_control,"size error");
        i = strlen((char *)input_control);
        cTBC_put_bypass_data(input_control, i);
        return;
    }
    memcpy(input_control, data, 9);
        
    m_bypass_data.mode = input_control[2];
    for(i=4, j=i;i< size;i++)
    {
      if(input_control[i]==',')
      {
        hexdigit_to_hexnum(&m_bypass_data.reg,&input_control[j],i-j);
        hexdigit_to_hexnum(&m_bypass_data.value,&input_control[i+1],size-i-1);
      }
    }
}



#endif


void bma250_power_on()
{
    nrf_gpio_cfg_output(26);
    nrf_gpio_pin_write(26, 1);
    nrf_gpio_cfg_output(24);
    nrf_gpio_pin_write(24, 1);
}

bma250_state_s bma250_get_state()
{
    return m_bma250_state;
}

void bma250_set_state(bma250_state_s m_state)
{
    m_bma250_state = m_state;
}



void bma250_i2c_bus_write(u8 reg_addr, u8 *reg_data, u8 cnt)
{

    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BMA2x2_INIT_VALUE;

    array[BMA2x2_INIT_VALUE] = reg_addr;
        for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) 
            array[stringpos + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] =
            *(reg_data + stringpos);
 
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BMA2x2_I2C_ADDR1,array, cnt+1, false));
}


void bma250_i2c_bus_read(u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 stringpos = BMA2x2_INIT_VALUE;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    /* Please take the below function as your reference
     * for read the data using I2C communication
     * add your I2C rad function here.
     * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
     * iError is an return value of SPI write function
     * Please select your valid return value
     * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
     */
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BMA2x2_I2C_ADDR1,&reg_addr, 1, false));
    while((!spi_xfer_done) && --timeout);

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi,BMA2x2_I2C_ADDR1, reg_data, cnt));
    while((!spi_xfer_done) && --timeout);
    for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++)
        *(reg_data + stringpos) = reg_data[stringpos];

}

uint32_t acc_bypass_write(u8 reg_addr, u8 * reg_data)
{
    uint32_t timeout;

    spi_xfer_done = false;
    m_tx_buf[0] = *reg_data;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(reg_addr,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    return NRF_SUCCESS;
}

uint32_t acc_bypass_read(u8 reg_addr, u8 cnt)
{
    uint32_t timeout;

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    bma250_i2c_bus_read(reg_addr,m_rx_buf,cnt);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    return NRF_SUCCESS;
}
void bma250_read_chip()
{
    memset(m_rx_buf,0x00,sizeof(m_rx_buf));
    bma250_i2c_bus_read(BMA2x2_CHIP_ID_REG,m_rx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);

}

uint32_t cfg_bma250_ISR_pin_test(void)
{
    uint32_t timeout;
   
    // softreset
    spi_xfer_done = false;
    m_tx_buf[0] = 0xB6;
    timeout = MPU_TWI_TIMEOUT;
    bma250_i2c_bus_write(BMA2x2_RST_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    nrf_delay_ms(10);

    //INT pin to Low
    spi_xfer_done = false;
    m_tx_buf[0] = 0x04;
    timeout = MPU_TWI_TIMEOUT;
    bma250_i2c_bus_write(BMA2x2_INTR_SET_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    return NRF_SUCCESS;
}

uint32_t cfg_bma250_sw_reset(void)
{
    uint32_t timeout;
   
    // softreset
    spi_xfer_done = false;
    m_tx_buf[0] = 0xB6;
    timeout = MPU_TWI_TIMEOUT;
    bma250_i2c_bus_write(BMA2x2_RST_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    nrf_delay_ms(10);
    return NRF_SUCCESS;
}

uint32_t bma250_req_suppend_mode(void)
{
    uint32_t timeout;
    spi_xfer_done = false;
    m_tx_buf[0] = 0x80;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_MODE_CTRL_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    return NRF_SUCCESS;
}

uint32_t bma250_slope_set()
{
    uint32_t timeout;

#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2) || (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
    spi_xfer_done = false;
    m_tx_buf[0] = 0xB6;
    timeout = MPU_TWI_TIMEOUT;
    // softreset
    bma250_i2c_bus_write(BMA2x2_RST_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    nrf_delay_ms(10);

    spi_xfer_done = false;
    m_tx_buf[0] = 0x5C;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_MODE_CTRL_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2) || (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
    spi_xfer_done = false;
    m_tx_buf[0] = BMA2x2_RANGE_2G;
    timeout = MPU_TWI_TIMEOUT;
    //  select the accelerometer g-range 
    bma250_i2c_bus_write(BMA2x2_RANGE_SELECT_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = 0x07;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_INTR_ENABLE1_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = 0x30;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_SLOPE_THRES_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = 0x01;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_SLOPE_DURN_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;


    spi_xfer_done = false;
    m_tx_buf[0] = 0x04;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_INTR1_PAD_SELECT_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = 0x5C;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_MODE_CTRL_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;  
#endif
#else
    spi_xfer_done = false;
    m_tx_buf[0] = 0xB6;
    timeout = MPU_TWI_TIMEOUT;
    // softreset
    bma250_i2c_bus_write(BMA2x2_RST_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    nrf_delay_ms(10);

    spi_xfer_done = false;
    m_tx_buf[0] = 0x5C;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_MODE_CTRL_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = BMA2x2_RANGE_2G;
    timeout = MPU_TWI_TIMEOUT;
    //  select the accelerometer g-range 
    bma250_i2c_bus_write(BMA2x2_RANGE_SELECT_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    /*
             spi_xfer_done = false;
             m_tx_buf[0] = BMA2x2_BW_1000HZ;
             timeout = MPU_TWI_TIMEOUT;
        
             bma250_i2c_bus_write(BMA2x2_BW_REG,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
             while((!spi_xfer_done) && --timeout);
             if(!timeout) return NRF_ERROR_TIMEOUT;
    */

    spi_xfer_done = false;
    m_tx_buf[0] = 0x07;
    timeout = MPU_TWI_TIMEOUT;
    // control which interrupt engines in group 1
    bma250_i2c_bus_write(BMA2x2_INTR_ENABLE2_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

/*
        spi_xfer_done = false;
        m_tx_buf[0] = 0x00; //0x50;
        timeout = MPU_TWI_TIMEOUT;
        // contains the low-g interrupt hysteresis setting and the high-g interrupt hysteresis setting
        bma250_i2c_bus_write(BMA2x2_LOW_HIGH_HYST_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
        while((!spi_xfer_done) && --timeout);
        if(!timeout) return NRF_ERROR_TIMEOUT;
        
*/
            spi_xfer_done = false;
            m_tx_buf[0] = 0xFF;
            timeout = MPU_TWI_TIMEOUT;
        // slope thread
            bma250_i2c_bus_write(BMA2x2_HIGH_THRES_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
            while((!spi_xfer_done) && --timeout);
            if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = 0x01;
    timeout = MPU_TWI_TIMEOUT;
    // contains the delay time definition for the high-g interrupt
    bma250_i2c_bus_write(BMA2x2_HIGH_DURN_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    nrf_delay_ms(10);
    spi_xfer_done = false;
    m_tx_buf[0] = 0x02;
    timeout = MPU_TWI_TIMEOUT;
    // controls which interrupt signals are mapped to the INT1 pin
    bma250_i2c_bus_write(BMA2x2_INTR1_PAD_SELECT_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;




    spi_xfer_done = false;
    m_tx_buf[0] = 0x5C;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_MODE_CTRL_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;


#endif

    return NRF_SUCCESS;
}

uint32_t bma250_high_set()
{
    uint32_t timeout;

    spi_xfer_done = false;
    m_tx_buf[0] = 0xB6;
    timeout = MPU_TWI_TIMEOUT;
    // softreset
    bma250_i2c_bus_write(BMA2x2_RST_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    nrf_delay_ms(10);

    spi_xfer_done = false;
    m_tx_buf[0] = 0x5C;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_MODE_CTRL_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = BMA2x2_RANGE_2G;
    timeout = MPU_TWI_TIMEOUT;
    //  select the accelerometer g-range 
    bma250_i2c_bus_write(BMA2x2_RANGE_SELECT_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    /*
    spi_xfer_done = false;
    m_tx_buf[0] = BMA2x2_BW_1000HZ;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_BW_REG,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    */      
    spi_xfer_done = false;
    m_tx_buf[0] = 0x07;
    timeout = MPU_TWI_TIMEOUT;
    // control which interrupt engines in group 1
    bma250_i2c_bus_write(BMA2x2_INTR_ENABLE2_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    m_tx_buf[0] = 0x40;
    timeout = MPU_TWI_TIMEOUT;
    // contains the low-g interrupt hysteresis setting and the high-g interrupt hysteresis setting
    bma250_i2c_bus_write(BMA2x2_LOW_HIGH_HYST_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    /*
    spi_xfer_done = false;
    m_tx_buf[0] = 0x01;
    timeout = MPU_TWI_TIMEOUT;
    // slope thread
    bma250_i2c_bus_write(BMA2x2_HIGH_THRES_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    */
    spi_xfer_done = false;
    m_tx_buf[0] = 0x01;
    timeout = MPU_TWI_TIMEOUT;
    // contains the delay time definition for the high-g interrupt
    bma250_i2c_bus_write(BMA2x2_HIGH_DURN_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;


    nrf_delay_ms(10);
    spi_xfer_done = false;
    m_tx_buf[0] = 0x02;
    timeout = MPU_TWI_TIMEOUT;
    // controls which interrupt signals are mapped to the INT1 pin
    bma250_i2c_bus_write(BMA2x2_INTR1_PAD_SELECT_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;


    spi_xfer_done = false;
    m_tx_buf[0] = 0x5C;
    timeout = MPU_TWI_TIMEOUT;

    bma250_i2c_bus_write(BMA2x2_MODE_CTRL_ADDR,m_tx_buf,BMA2x2_GEN_READ_WRITE_LENGTH);
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT; 

    return NRF_SUCCESS;
}

void bma250_read_sensor()
{
    memset(m_rx_buf,0x00,sizeof(m_rx_buf));
//  bma250_SPI_bus_read(BMA2x2_ACCEL_X12_LSB_REG,m_rx_buf,BMA2x2_SHIFT_SIX_BITS);

    bma250_i2c_bus_read(BMA2x2_ACCEL_X12_LSB_REG,m_rx_buf,BMA2x2_SHIFT_SIX_BITS);

}

void bma250_read_accel_xyz(struct bma_accel_data *accel)
{
    u8 data_u8[BMA2x2_ACCEL_XYZ_DATA_SIZE] = {
    BMA2x2_INIT_VALUE, BMA2x2_INIT_VALUE,
    BMA2x2_INIT_VALUE, BMA2x2_INIT_VALUE,
    BMA2x2_INIT_VALUE, BMA2x2_INIT_VALUE};

    memcpy(data_u8,m_rx_buf,6);
    accel->x = (s16)((((s32)((s8)
    data_u8[BMA2x2_SENSOR_DATA_XYZ_X_MSB]))
    << BMA2x2_SHIFT_EIGHT_BITS) |
    (data_u8[BMA2x2_SENSOR_DATA_XYZ_X_LSB] &
    BMA2x2_10_BIT_SHIFT));
    accel->x = accel->x >> BMA2x2_SHIFT_SIX_BITS;

    /* read the y data_u8*/
    accel->y = (s16)((((s32)((s8)
    data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_MSB]))
    << BMA2x2_SHIFT_EIGHT_BITS) |
    (data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_LSB] &
    BMA2x2_10_BIT_SHIFT));
    accel->y = accel->y >> BMA2x2_SHIFT_SIX_BITS;

    /* read the z data_u8*/
    accel->z = (s16)((((s32)((s8)
    data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_MSB]))
    << BMA2x2_SHIFT_EIGHT_BITS) |
    (data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_LSB] &
    BMA2x2_10_BIT_SHIFT));
    accel->z = accel->z >> BMA2x2_SHIFT_SIX_BITS;
}

bool cfg_bma250_read_xyz(struct bma_accel_data *maccel)
{
    uint32_t timeout= MPU_TWI_TIMEOUT;

    struct bma_accel_data in_accel;
    spi_xfer_done = false;
    bma250_read_sensor();

    while((!spi_xfer_done) && --timeout);
    if(!timeout) return false;

    bma250_read_accel_xyz(&in_accel);
    maccel->x = in_accel.x;
    maccel->y = in_accel.y;
    maccel->z = in_accel.z;
    return true;
}

bool cfg_bma250_read_reg(uint8_t reg_addr,uint8_t * read)
{
    if(acc_bypass_read(reg_addr,(u8)1)==NRF_SUCCESS)
    {
        *read = m_rx_buf[0];
        return true;
    }
    else
    {
        return false;
    }
}

bool cfg_bma250_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    if(acc_bypass_write(reg_addr,&reg_data)==NRF_SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t position[50];
bool need_setup = false;
#ifdef FEATURE_CFG_ACC_REPORT
extern bool main_schedule_state_is_idle(void);
extern void nus_send_data(char module);
#endif
bool mnus_acc_report = false;
static void bma250_state_handler(void * p_context)
{
    static uint32_t error_code;

    switch(m_bma250_state)
    {
        case IO_SETUP:
            if(mnus_acc_report)
            {
                cfg_i2c_master_uninit();
                cfg_i2c_master_init();
                nrf_delay_ms(1);
                m_bma250_state = READ_DATA_S;
            }
            else
            {
                cfg_i2c_master_uninit();
                m_bma250_state = NONE_ACC;
            }
            break;
        case SET_S:
            spi_xfer_done = false;
            error_code = bma250_slope_set();
//            error_code = bma250_high_set();

            m_bma250_state = SET_R;
            break;   
        case SET_R:
            if(error_code == NRF_SUCCESS)
            {
                cPrintLog(CDBG_GSEN_INFO, "G-SENSOR SET \n");
                m_bma250_state = NONE_ACC;
                spi_xfer_done = false;
            }
            else
            {
                m_bma250_state = NONE_ACC;
            }
            break;

        case READ_DATA_S:
#ifdef FEATURE_CFG_ACC_REPORT
            if(main_schedule_state_is_idle()||cTBC_bypass_mode_is_accbypass())
#endif
            {
                if(m_cfg_i2c_master_init_flag)
                {
                    spi_xfer_done = false;
                    bma250_read_sensor();
                    m_bma250_state = READ_DATA_R;
                }
                else
                {
                    m_bma250_state = IO_SETUP;
                }
            }
            break;
        case READ_DATA_R:
            if(spi_xfer_done)
            {
                memset(position,0x00,50);
                bma250_read_accel_xyz(&m_accel);
#ifdef FEATURE_CFG_ACC_REPORT
                if(mnus_acc_report)
                {
                    m_nus_service_parameter.module = 'A';
                    m_nus_service_parameter.acc_x[0] = (m_accel.x &0XFF00)>>8;
                    m_nus_service_parameter.acc_x[1] = m_accel.x &0x00FF;
                    m_nus_service_parameter.acc_y[0] = (m_accel.y &0XFF00)>>8;
                    m_nus_service_parameter.acc_y[1] = m_accel.y &0x00FF;
                    m_nus_service_parameter.acc_z[0] = (m_accel.z &0XFF00)>>8;
                    m_nus_service_parameter.acc_z[1] =  m_accel.z &0x00FF;
                    nus_send_data('A');
                }
#endif
                /*
                if(abs(m_accel.x)+abs(m_accel.y)+abs(m_accel.z) > threadhold_value)
                {
                    main_wakeup_interrupt = true;
                }
                */
 //               cPrintLog(CDBG_GSEN_INFO, "s16 size :%d\n", sizeof(s16));
                cPrintLog(CDBG_GSEN_INFO,"X=%+d  Y=%+d  Z=%+d \n", m_accel.x, m_accel.y, m_accel.z);
                
//                cPrintLog(CDBG_GSEN_INFO, "%s\n", position);
//                sprintf((char *)position,"ACC Y = %+06d ", (float)m_accel.y/10);
//                cPrintLog(CDBG_GSEN_INFO, "%s %s\n", position);
//                sprintf((char *)position,"ACC Z = %+06d ", (float)m_accel.z/10);
//                cPrintLog(CDBG_GSEN_INFO, "%s %s\n", position);
 //                 m_bma250_state = BYPASS_S;
//                if( m_accel.x> 400 )
//                {
//                    main_wakeup_interrupt = true;
//                    cPrintLog(CDBG_GSEN_INFO, "%s SHAKE WAKE-UP\n", __func__);
//                }
//                nrf_delay_ms(1000);
#ifdef FEATURE_CFG_ACC_REPORT
                if(cTBC_bypass_mode_is_accbypass())
                {
                    m_bma250_state = BYPASS_R;
                }
                else
                {
                    m_bma250_state = READ_DATA_S;
                }
#else
                m_bma250_state = BYPASS_R;
#endif
                spi_xfer_done = false;
            }
            break;
        case SLEEP_S:
            m_bma250_state = SLEEP_R;
            break;

        case SLEEP_R:
            m_bma250_state = EXIT_ACC;
            break;
        case BYPASS_S:
            if(m_bypass_data.mode =='r'&&m_bypass_data.reg!=0xff)
            {
                error_code = acc_bypass_read(m_bypass_data.reg,(u8)m_bypass_data.value);
                m_bma250_state = BYPASS_R;
            }
            else if(m_bypass_data.mode =='w')
            {
                error_code = acc_bypass_write(m_bypass_data.reg,(u8 *)&m_bypass_data.value);
                m_bma250_state = BYPASS_R;
            }
            else if(m_bypass_data.mode =='r'&&m_bypass_data.reg==0xff)
            {
                memset(&m_bypass_data,0x00,sizeof(struct bma2x2_bypass_data));
                m_bma250_state = READ_DATA_S;
            }
            break;
        case BYPASS_R:
            if(error_code == NRF_SUCCESS)
            {
                if(m_bypass_data.mode =='r')
                {
                    sprintf((char *)position," READ REG:%x VAL:%x", m_bypass_data.reg,m_rx_buf[0]);
                    m_bypass_data.value = (uint8_t)strlen((char *)position);
                    cTBC_put_bypass_data(position, m_bypass_data.value);
                }
                else if(m_bypass_data.mode =='w')
                {
                    sprintf((char *)position," WRITE REG:%x VAL:%x", m_bypass_data.reg,m_bypass_data.value);
                    m_bypass_data.value = (uint8_t)strlen((char *)position);
                    cTBC_put_bypass_data(position, m_bypass_data.value);
                }
                memset(&m_bypass_data,0x00,sizeof(struct bma2x2_bypass_data));

                m_bma250_state = BYPASS_S;

                spi_xfer_done = false;
            }
            else
            {
                memset(&m_bypass_data,0x00,sizeof(struct bma2x2_bypass_data));
                m_bma250_state = BYPASS_S;
            }
            break;
        case EXIT_ACC:
            break;
        default:
            break;
    }
}

void cfg_bma250_timer_create()
{
    uint32_t err_code;


    // Create timers.
    err_code = app_timer_create(&m_bma250_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                bma250_state_handler);
    APP_ERROR_CHECK(err_code);
}

void cfg_bma250_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    if(mnus_acc_report)
    {
        err_code = app_timer_start(m_bma250_timer_id, APP_TIMER_TICKS(25, APP_TIMER_PRESCALER), NULL);
    }
    else
    {
        err_code = app_timer_start(m_bma250_timer_id, APP_TIMER_TICKS(200, APP_TIMER_PRESCALER), NULL);
    }
    APP_ERROR_CHECK(err_code);
}
void cfg_bma250_timers_stop(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_stop(m_bma250_timer_id);
    APP_ERROR_CHECK(err_code);
}

