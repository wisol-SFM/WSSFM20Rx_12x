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
 * @brief tracking Sample Application cfg_board.c file.
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
#include "nrf_drv_twi.h"

#define CDBG_LOG_INSTANCE
#include "cfg_dbg_log.h"
#include "cfg_board_def.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_twis_board_control.h"
#include "cfg_board.h"
#include "cfg_external_sense_gpio.h"
#include "cfg_sigfox_module.h"

const char m_cfg_sw_ver[4] = {CDEV_SW_VER_MAJOR CDEV_SW_VER_MINOR};
const char m_cfg_model_name[] = {CDEV_MODEL_NAME};
const char m_cfg_board_type = CDEV_BOARD_TYPE;
const char m_cfg_build_date[] = __DATE__;
const char m_cfg_build_time[] = __TIME__;

bool m_cfg_sw_reset_detected = false;
bool m_cfg_debug_interface_wake_up_detected = false;
bool m_cfg_available_bootloader_detected = false;
bool m_cfg_NFC_wake_up_detected = false;
bool m_cfg_GPIO_wake_up_detected = false;
bool m_cfg_i2c_master_init_flag = false;

int m_cfg_comm_pwr_mask = 0;

const nrf_drv_spi_config_t m_spi_config_default = NRF_DRV_SPI_DEFAULT_CONFIG;
extern int dtm_mode(void);

//util function
void cfg_bin_2_hexadecimal(const uint8_t *pBin, int binSize, char *pHexadecimal)
{
    uint8_t data, dH, dL;
    int i;

    for(i = 0; i < binSize; i++)
    {
        data = *pBin++;
        dH = (data >> 4);
        dL = (data & 0x0F);
        if(dH < 10)
            *pHexadecimal++ = ('0'+ dH);
        else
            *pHexadecimal++ = ('A'+ (dH-10));
        if(dL < 10)
            *pHexadecimal++ = ('0'+ dL);
        else
            *pHexadecimal++ = ('A'+ (dL-10));
    }
    *pHexadecimal = 0;
}

uint32_t cfg_hexadecimal_2_uint_val_big(const char *pHexadecimal, int len)
{
    uint32_t uint_val = 0;
    int i = 0;

    for(i=0; i<len; i++)
    {
        uint_val = (uint_val << 4);
        if(pHexadecimal[i] >= '0' && pHexadecimal[i] <= '9')
        {
            uint_val += (pHexadecimal[i]-'0');
        }
        else if(pHexadecimal[i] >= 'A' && pHexadecimal[i] <= 'F')
        {
            uint_val += ((pHexadecimal[i]-'A')+10);
        }
        else if(pHexadecimal[i] >= 'a' && pHexadecimal[i] <= 'f')
        {
            uint_val += ((pHexadecimal[i]-'a')+10);
        }
    }
    return uint_val;
}

void cfg_hexadecimal_2_bin(const char *pHexadecimal, int HexadeccoimalByteCnt, uint8_t *pBin)
{
    int i;
    uint8_t h_val, l_val;
    if((HexadeccoimalByteCnt < 0) || ((HexadeccoimalByteCnt % 2) != 0))
        return;

    for(i = 0; i < HexadeccoimalByteCnt; i+=2)
    {
        h_val = 0;
        l_val = 0;
        if(pHexadecimal[i] >= 'A' && pHexadecimal[i] <= 'F')
            h_val = pHexadecimal[i]-'A'+10;
        else if(pHexadecimal[i] >= 'a' && pHexadecimal[i] <= 'f')
            h_val = pHexadecimal[i]-'a'+10;
        else if(pHexadecimal[i] >= '0' && pHexadecimal[i] <= '9')
            h_val = pHexadecimal[i]-'0';
        
        if(pHexadecimal[i+1] >= 'A' && pHexadecimal[i+1] <= 'F')
            l_val = pHexadecimal[i+1]-'A'+10;
        else if(pHexadecimal[i+1] >= 'a' && pHexadecimal[i+1] <= 'f')
            l_val = pHexadecimal[i+1]-'a'+10;
        else if(pHexadecimal[i+1] >= '0' && pHexadecimal[i+1] <= '9')
            l_val = pHexadecimal[i+1]-'0';

        *pBin++ = (h_val << 4 | l_val);
    }
}

//convert the string to number
int cfg_atoi(const char *str)
{
    int num = 0, i = 0;
    int val = 1;

    if( NULL == str)
    {
        return 0;
    }
    if(str[i]=='-')
    {
        i++;
        val = -1;
    }
    while(str[i] != '\0')
    {
        if(str[i] >= '0' && str[i] <= '9')
        {
            num = num * 10 + (str[i] - '0');
            i++;
        }
        else
        {
            break;
        }
    }
    return (num * val);
}

void cfg_board_common_power_control(module_comm_pwr_resource_e resource, bool bOn)
{
    if(bOn)
    {
        if(m_cfg_comm_pwr_mask == 0)
        {
            nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
            nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 1);
            nrf_delay_ms(2);  //spec is 2uS
        }
        m_cfg_comm_pwr_mask = (m_cfg_comm_pwr_mask | (0x01<<resource));
    }
    else
    {
        m_cfg_comm_pwr_mask = (m_cfg_comm_pwr_mask & ~(0x01<<resource));
        if(m_cfg_comm_pwr_mask == 0)
        {
            nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
            nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);
            nrf_delay_ms(1);
        }
    }
}

static bool old_ble_led_status = false;
bool cfg_get_ble_led_status(void)
{
    return old_ble_led_status;
}

bool cfg_ble_led_control(bool bOn)
{
    static bool bGpioInit = false;
    bool ret;
    if(!bGpioInit)
    {
        nrf_gpio_cfg_output(PIN_DEF_BLE_LED_EN);
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 0);
        bGpioInit = true;
    }
    ret = old_ble_led_status;
    old_ble_led_status = bOn;
    if(bOn)
    {
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 1);
    }
    else
    {
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 0);
    }
    return ret;
}

bool cfg_is_3colorled_contorl(void)
{
    if(m_module_parameter.wkup_gpio_enable==2)
        return true;
    else
        return false;
}

void cfg_wkup_output_control(bool bOn)  // ihere 3color led reference m_module_parameter.wkup_gpio_enable==2
{
    static bool bGpioInit = false;

    if(cfg_is_3colorled_contorl())
    {
        if(!bGpioInit)
        {
            nrf_gpio_cfg_output(PIN_DEF_WKUP);
            nrf_gpio_pin_write(PIN_DEF_WKUP, 0);
            bGpioInit = true;
        }
        if(bOn)
        {
            nrf_gpio_pin_write(PIN_DEF_WKUP, 1);
        }
        else
        {
            nrf_gpio_pin_write(PIN_DEF_WKUP, 0);
        }
    }
}

const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(GSEN_TWI_INSTANCE);
/* Indicates if operation on TWI has ended. */
volatile bool spi_xfer_done = false;

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch (p_event->xfer_desc.type )
            {
                case NRF_DRV_TWI_XFER_TX :
                    spi_xfer_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX: 
                    spi_xfer_done = true; 
                    break; 
                case NRF_DRV_TWI_XFER_RX: 
                    spi_xfer_done = true; 
                    break; 
                case NRF_DRV_TWI_XFER_TXRX: 
                    spi_xfer_done = true; 
                    break; 
                default: 
                    break; 
            }
            break;
        default:
            break;
    }
}

/**@brief Function for initializing I2C in accelerometer.
 *                         
 */
void cfg_i2c_master_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = PIN_DEF_ACC_TWIS_SCL,
       .sda                = PIN_DEF_ACC_TWIS_SDA,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    m_cfg_i2c_master_init_flag = true;
}
/**@brief Function for uninitializing I2C in accelerometer.
 *                         
 */

void cfg_i2c_master_uninit(void)
{
    nrf_drv_twi_disable(&m_twi);
    nrf_drv_twi_uninit(&m_twi);
    m_cfg_i2c_master_init_flag = false;
}

static void cfg_board_resource_init(void)
{
}

void cfg_board_reset(void)
{
    NVIC_SystemReset();
}

void cfg_board_check_reset_reason(void)
{
    unsigned int reset_reason = NRF_POWER->RESETREAS;

    if(reset_reason & POWER_RESETREAS_RESETPIN_Msk)  //reset pin or power on reset
    {
        NRF_POWER->RESETREAS = POWER_RESETREAS_RESETPIN_Msk | POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DIF_Msk | POWER_RESETREAS_NFC_Msk | POWER_RESETREAS_OFF_Msk;
    }
    else
    {
        if(reset_reason & POWER_RESETREAS_SREQ_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "sw reset det\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_SREQ_Msk;
            m_cfg_sw_reset_detected = true;
        }

        if(reset_reason & POWER_RESETREAS_DIF_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "debug interface wake up\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_DIF_Msk;
            m_cfg_debug_interface_wake_up_detected = true;
        }

        if(reset_reason & POWER_RESETREAS_NFC_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "NFC wake up det\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_NFC_Msk;
            m_cfg_NFC_wake_up_detected = true;
        }

        if(reset_reason & POWER_RESETREAS_OFF_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "GPIO wake up det\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_OFF_Msk;
            m_cfg_GPIO_wake_up_detected = true;
        }
    }

}

void cfg_board_check_bootloader(void)
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    if(bootloader_addr == WISOL_BOOTLOADER_RESERVED_END_ADDR)
    {
        m_cfg_available_bootloader_detected = true;
    }
}

void cfg_board_gpio_set_default_gps(void)
{

#ifdef CDEV_GPS_MODULE
    nrf_gpio_cfg_output(PIN_DEF_GPS_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 0);
    nrf_gpio_cfg_output(PIN_DEF_GPS_RESET);
    nrf_gpio_pin_write(PIN_DEF_GPS_RESET, 0);

    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MISO);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_SCK);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_CS);
#endif
}

void cfg_board_gpio_set_default(void)
{
#ifdef CDEV_WIFI_MODULE
    nrf_gpio_cfg_output(PIN_DEF_WIFI_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_WIFI_PWR_EN, 0);
    nrf_gpio_cfg_output(PIN_DEF_WIFI_RESET);
    nrf_gpio_pin_write(PIN_DEF_WIFI_RESET, 0);

    nrf_gpio_cfg_default(PIN_DEF_WIFI_INT);

    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_MISO);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_CLK);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_CS);
#endif

#if 0 // def CDEV_GPS_MODULE  // temp
    nrf_gpio_cfg_output(PIN_DEF_GPS_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 0);
    nrf_gpio_cfg_output(PIN_DEF_GPS_RESET);
    nrf_gpio_pin_write(PIN_DEF_GPS_RESET, 0);

    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MISO);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_SCK);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_CS);
#endif

    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 0);
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
    
    nrf_gpio_cfg_default(PIN_DEF_SIGFOX_UART_TX);
    nrf_gpio_cfg_default(PIN_DEF_SIGFOX_UART_RX);

    nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
    nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);
    
    nrf_gpio_cfg_default(PIN_DEF_TWIS_BOARD_CTRL_SCL);
    nrf_gpio_cfg_default(PIN_DEF_TWIS_BOARD_CTRL_SDA);

    nrf_gpio_cfg_default(PIN_DEF_ACC_TWIS_SCL);
    nrf_gpio_cfg_default(PIN_DEF_ACC_TWIS_SDA);
    nrf_gpio_cfg_default(PIN_DEF_ACC_INT1);

    nrf_gpio_cfg_default(PIN_DEF_MAGNETIC_SIGNAL);
    nrf_gpio_cfg_default(PIN_DEF_WKUP);

    nrf_gpio_cfg_output(PIN_DEF_BLE_LED_EN);
    nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 0);
    
}

static void cfg_board_testmode_gps_wifi_download(void)
{
    bool led_on_off = false;
    int tickCnt = 0;

    cPrintLog(CDBG_FCTRL_INFO, "====enter download mdoe====\n");
#ifdef CDEV_GPS_MODULE
#ifdef CDEV_CSR_GPS_MODULE
    nrf_gpio_cfg_input(PIN_DEF_GPS_SPI1_SCK, NRF_GPIO_PIN_PULLUP);
#endif /* CDEV_CSR_GPS_MODULE */
    cGps_gpio_init();
    cGps_download_power_control();  // for gps test mode
#endif /* CDEV_GPS_MODULE */

#ifdef CDEV_WIFI_MODULE
    cWifi_enter_download_mode();
#endif
    while(1)
    {
        cPrintLog(CDBG_FCTRL_INFO, "gps and wifi download mode! %d\n", tickCnt++);
        (void)tickCnt;
        cfg_ble_led_control(led_on_off);
        led_on_off = !led_on_off;
        nrf_delay_ms(2000);
    }
}

static void cfg_board_testmode_wifi(void)
{
    bool led_on_off = false;
    int tickCnt = 0;

    cPrintLog(CDBG_FCTRL_INFO, "====enter wifi test mode (always on)====\n");
#ifdef CDEV_WIFI_MODULE
    cWifi_enter_rftest_mode();
#endif
    while(1)
    {
        cPrintLog(CDBG_FCTRL_INFO, "testmode_wifi %d\n", tickCnt++);
        (void)tickCnt;
        cfg_ble_led_control(led_on_off);
        led_on_off = !led_on_off;
        nrf_delay_ms(5000);
    }
}

static void cfg_board_testmode_ble(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "====enter ble dtm mode====\n");
    dtm_mode();
}

#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
static void cfg_board_check_bootstrap_pin(void)
{
    uint32_t pinLvl_DL_EN, pinLvl_I2C0_SCL_DBG,pinLvl_I2C0_SDA_DBG;
    uint32_t bootstrap_config0, bootstrap_config1;

    uint32_t pinLvl_I2C0_SCL_DBG_old, pinLvl_I2C0_SDA_DBG_old;
    bool special_mode = false;
    int i;

    nrf_gpio_cfg_input(PIN_DEF_WIFI_INT, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(PIN_DEF_TWIS_BOARD_CTRL_SCL, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(PIN_DEF_TWIS_BOARD_CTRL_SDA, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(50);

    pinLvl_DL_EN = nrf_gpio_pin_read(PIN_DEF_WIFI_INT);

    pinLvl_I2C0_SCL_DBG = nrf_gpio_pin_read(PIN_DEF_TWIS_BOARD_CTRL_SCL);
    pinLvl_I2C0_SDA_DBG = nrf_gpio_pin_read(PIN_DEF_TWIS_BOARD_CTRL_SDA);
    pinLvl_I2C0_SCL_DBG_old = pinLvl_I2C0_SCL_DBG;
    pinLvl_I2C0_SDA_DBG_old = pinLvl_I2C0_SDA_DBG;

    //i2c slave busy check
    for(i = 0; i<100; i++)
    {
        nrf_delay_us(1);
        pinLvl_I2C0_SCL_DBG = nrf_gpio_pin_read(PIN_DEF_TWIS_BOARD_CTRL_SCL);
        pinLvl_I2C0_SDA_DBG = nrf_gpio_pin_read(PIN_DEF_TWIS_BOARD_CTRL_SDA);
        if((pinLvl_I2C0_SCL_DBG_old != pinLvl_I2C0_SCL_DBG) || (pinLvl_I2C0_SDA_DBG_old != pinLvl_I2C0_SDA_DBG))
        {
            //i2c slave connected and working
            cPrintLog(CDBG_FCTRL_INFO, "bootstrap pin i2c slave connected and working\n");
            pinLvl_I2C0_SCL_DBG = 1;
            pinLvl_I2C0_SDA_DBG = 1;
            break;
        }
        pinLvl_I2C0_SCL_DBG_old = pinLvl_I2C0_SCL_DBG;
        pinLvl_I2C0_SDA_DBG_old = pinLvl_I2C0_SDA_DBG;
    }
    if((pinLvl_I2C0_SCL_DBG == 0 && pinLvl_I2C0_SDA_DBG == 1) || (pinLvl_I2C0_SCL_DBG == 1 && pinLvl_I2C0_SDA_DBG == 0))
    {
        special_mode = true;
    }
    nrf_gpio_cfg_default(PIN_DEF_WIFI_INT);
    nrf_gpio_cfg_default(PIN_DEF_TWIS_BOARD_CTRL_SCL);
    nrf_gpio_cfg_default(PIN_DEF_TWIS_BOARD_CTRL_SDA);

    cPrintLog(CDBG_FCTRL_INFO, "bootstrap pin lvl: dl:%d, scl:%d, sda:%d\n", pinLvl_DL_EN, pinLvl_I2C0_SCL_DBG, pinLvl_I2C0_SDA_DBG);

    if(pinLvl_DL_EN == 0)
    {
        cfg_board_testmode_gps_wifi_download();
    }
    else if(special_mode)
    {
        if(pinLvl_I2C0_SCL_DBG == 0 && pinLvl_I2C0_SDA_DBG == 1)
        {
            nrf_gpio_cfg_input(PIN_DEF_BOOTSTRAP_CONFIG0, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(PIN_DEF_BOOTSTRAP_CONFIG1, NRF_GPIO_PIN_PULLDOWN);
            nrf_delay_ms(20);
            bootstrap_config0 = nrf_gpio_pin_read(PIN_DEF_BOOTSTRAP_CONFIG0);
            bootstrap_config1 = nrf_gpio_pin_read(PIN_DEF_BOOTSTRAP_CONFIG1);
            nrf_gpio_cfg_default(PIN_DEF_BOOTSTRAP_CONFIG0);
            nrf_gpio_cfg_default(PIN_DEF_BOOTSTRAP_CONFIG1);
            cPrintLog(CDBG_FCTRL_INFO, "bootstrap scl-0 sda-1 cfg1:%d, cfg2:%d\n", bootstrap_config0, bootstrap_config1);
            if(bootstrap_config0 == 0 && bootstrap_config1 == 0)
            {
                //not assigned
            }
            else if(bootstrap_config0 == 0 && bootstrap_config1 == 1)
            {
                cfg_board_testmode_ble();
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 0)
            {
                cfg_board_testmode_wifi();
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 1)
            {
                //not assigned
            }
        }
        else if(pinLvl_I2C0_SCL_DBG == 1 && pinLvl_I2C0_SDA_DBG == 0)
        {
            //not assigned
        }
    }
}
#endif

void cfg_board_early_init(void)
{
    cfg_board_check_reset_reason();
#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
    cfg_board_check_bootstrap_pin();
#endif
    cfg_board_gpio_set_default();
    cfg_board_check_bootloader();
}

void cfg_board_init(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "%s\n", __func__);
    cfg_board_resource_init();

    cfg_board_common_power_control(module_comm_pwr_pon_init, true);

#ifdef CDEV_WIFI_MODULE
    cWifi_resource_init();
    cWifi_prepare_start();
#endif
    cfg_sigfox_prepare_start();

#ifdef CDEV_GPS_MODULE
    cGps_resource_init();
    cGps_prepare_start();
#endif /* CDEV_GPS_MODULE */
    cfg_board_common_power_control(module_comm_pwr_pon_init, false);
}

