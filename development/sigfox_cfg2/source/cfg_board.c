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
extern void set_CN0_check_type(int val);  /*for common lib*/
extern void set_CN0_enable(unsigned int val);  /*for common lib*/

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

#ifdef CDEV_RTC2_DATE_TIME_CLOCK
#define MAX_RTC2_TASKS_DELAY     47                                          /**< Maximum delay until an RTC task is executed. ref MAX_RTC_TASKS_DELAY*/
#define RTC2_TIMER_CONFIG_IRQ_PRIORITY 7  //ref APP_TIMER_CONFIG_IRQ_PRIORITY
#define DATE_TIME_CLOCK_DEFAULT (1483196400+32400)  //2017.01.01-00:00:00 based linux timestamp - max is 2038-01-19 03:14:07

static uint32_t date_time_clock_timestamp;
static bool date_time_clock_start_flag = false;

void RTC2_IRQHandler(void)
{
    cPrintLog(CDBG_COMMON_LOG, "Rtc2 IRQ occurred\n");

    date_time_clock_timestamp += 2097152;  //PRESCALER 4095 =  2097152 sec - about 582.542 hours //PRESCALER 0 is 512 sec
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->EVENTS_COMPARE[1] = 0;
    NRF_RTC2->EVENTS_COMPARE[2] = 0;
    NRF_RTC2->EVENTS_COMPARE[3] = 0;
    NRF_RTC2->EVENTS_TICK       = 0;
    NRF_RTC2->EVENTS_OVRFLW     = 0;
}

void RTC2_date_time_clock_init_N_start(void)  //date_time
{
    NRF_RTC2->PRESCALER = 4095;  //2097152 sec  //0 is 512 sec
    NVIC_SetPriority(RTC2_IRQn, RTC2_TIMER_CONFIG_IRQ_PRIORITY);

    NRF_RTC2->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
    NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    NVIC_ClearPendingIRQ(RTC2_IRQn);
    NVIC_EnableIRQ(RTC2_IRQn);

    date_time_clock_timestamp = DATE_TIME_CLOCK_DEFAULT;
    NRF_RTC2->TASKS_START = 1;
    nrf_delay_us(MAX_RTC2_TASKS_DELAY);
    date_time_clock_start_flag = true;
}

uint32_t date_time_get_timestamp(void)  //date_time
{
    uint32_t RTC2_counter;
    uint32_t timestamp = 0;

    if(date_time_clock_start_flag)
    {
        RTC2_counter = NRF_RTC2->COUNTER;
        timestamp = (date_time_clock_timestamp + (RTC2_counter / 8));
    }
    return timestamp;
}

void date_time_set_timestamp(uint32_t timestamp)
{
    uint32_t RTC2_counter;

    RTC2_counter = NRF_RTC2->COUNTER;
    if(timestamp < (RTC2_counter / 8))
    {
        date_time_clock_timestamp = 0;
    }
    else
    {
        date_time_clock_timestamp = timestamp - (RTC2_counter / 8);
    }
}

void date_time_get_current_time(cfg_date_time_t *tm)
{
  uint32_t seconds, minutes, hours, days, year, month;
  uint32_t dayOfWeek;
  seconds = date_time_get_timestamp();

  /* calculate minutes */
  minutes  = seconds / 60;
  seconds -= minutes * 60;
  /* calculate hours */
  hours    = minutes / 60;
  minutes -= hours   * 60;
  /* calculate days */
  days     = hours   / 24;
  hours   -= days    * 24;

  /* Unix time starts in 1970 on a Thursday */
  year      = 1970;
  dayOfWeek = 4;

  while(1)
  {
    bool     leapYear   = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    uint16_t daysInYear = leapYear ? 366 : 365;
    if (days >= daysInYear)
    {
      dayOfWeek += leapYear ? 2 : 1;
      days      -= daysInYear;
      if (dayOfWeek >= 7)
        dayOfWeek -= 7;
      ++year;
    }
    else
    {
      dayOfWeek  += days;
      dayOfWeek  %= 7;

      /* calculate the month and day */
      static const uint8_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      for(month = 0; month < 12; ++month)
      {
        uint8_t dim = daysInMonth[month];

        /* add a day to feburary if this is a leap year */
        if (month == 1 && leapYear)
          ++dim;

        if (days >= dim)
          days -= dim;
        else
          break;
      }
      break;
    }
  }

  tm->seconds  = seconds;
  tm->minutes  = minutes;
  tm->hours = hours;
  tm->day = days + 1;
  tm->month = month + 1;
  tm->year = year;
  tm->dayOfWeek = dayOfWeek;
}
#endif

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
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 1);
#else
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 0);
#endif
        bGpioInit = true;
    }
    ret = old_ble_led_status;
    old_ble_led_status = bOn;
    if(bOn)
    {
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 0);
#else
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 1);
#endif
    }
    else
    {
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 1);
#else
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 0);
#endif
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
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
            nrf_gpio_pin_write(PIN_DEF_WKUP, 1);
#else
            nrf_gpio_pin_write(PIN_DEF_WKUP, 0);
#endif
            bGpioInit = true;
        }
        if(bOn)
        {
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
            nrf_gpio_pin_write(PIN_DEF_WKUP, 0);
#else
            nrf_gpio_pin_write(PIN_DEF_WKUP, 1);
#endif
        }
        else
        {
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
            nrf_gpio_pin_write(PIN_DEF_WKUP, 1);
#else
            nrf_gpio_pin_write(PIN_DEF_WKUP, 0);
#endif
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
    
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
    nrf_gpio_cfg_output(PIN_DEF_WKUP);
    nrf_gpio_pin_write(PIN_DEF_WKUP, 1);
    nrf_gpio_cfg_output(PIN_DEF_BLE_LED_EN);
    nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 1);
#else
    nrf_gpio_cfg_default(PIN_DEF_WKUP);

    nrf_gpio_cfg_output(PIN_DEF_BLE_LED_EN);
    nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, 0);
#endif
    
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

#ifdef CDEV_WIFI_MODULE  //ESP8285
    nrf_gpio_cfg_input(PIN_DEF_WIFI_INT, NRF_GPIO_PIN_PULLUP);
#endif
    nrf_gpio_cfg_input(PIN_DEF_TWIS_BOARD_CTRL_SCL, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(PIN_DEF_TWIS_BOARD_CTRL_SDA, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(50);

#ifdef CDEV_WIFI_MODULE  //ESP8285
    pinLvl_DL_EN = nrf_gpio_pin_read(PIN_DEF_WIFI_INT);
#else
    pinLvl_DL_EN = 1;
#endif

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
#ifdef CDEV_WIFI_MODULE  //ESP8285
    nrf_gpio_cfg_default(PIN_DEF_WIFI_INT);
#endif
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
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
    set_CN0_check_type(2);  /*for common lib*/
    set_CN0_enable(CGPS_CNO_CHECK_ENABLE);  /*for common lib*/  //move to here
#endif
#endif /* CDEV_GPS_MODULE */
    cfg_board_common_power_control(module_comm_pwr_pon_init, false);
#ifdef CDEV_RTC2_DATE_TIME_CLOCK
    RTC2_date_time_clock_init_N_start();
#endif
}

