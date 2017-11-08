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
static bool m_cfg_bridge_from_uart_to_uart_flag = false;  //for sigfox uart bridge

int m_cfg_comm_pwr_mask = 0;
static uint32_t m_cfg_testmode_wait_tick;

const nrf_drv_spi_config_t m_spi_config_default = NRF_DRV_SPI_DEFAULT_CONFIG;

static uint8_t *m_cfg_board_testmode_tx_buf_uart;
static uint32_t m_cfg_board_testmode_tx_buf_uart_idx;
static uint8_t *m_cfg_board_testmode_rx_buf_uart;
static uint32_t m_cfg_board_testmode_rx_buf_uart_idx;
static uint8_t *m_cfg_board_testmode_rx_buf_rtt;  //down data to rtt
static uint32_t m_cfg_board_testmode_rx_buf_rtt_idx;  //down data to rtt
static uint8_t *m_cfg_board_testmode_tx_buf_rtt;  //up data to rtt
static uint32_t m_cfg_board_testmode_tx_buf_rtt_idx;  //up data to rtt
static uint8_t *m_cfg_board_testmode_tx_buf_uart_2nd;      //for sigfox uart bridge
static uint32_t m_cfg_board_testmode_tx_buf_uart_idx_2nd;  //for sigfox uart bridge
static uint8_t *m_cfg_board_testmode_rx_buf_uart_2nd;      //for sigfox uart bridge
static uint32_t m_cfg_board_testmode_rx_buf_uart_idx_2nd;  //for sigfox uart bridge

extern int dtm_mode(void);
extern void set_CN0_check_type(int val);  /*for common lib*/
extern void set_CN0_enable(unsigned int val);  /*for common lib*/
extern void main_examples_prepare(void);

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

static void cfg_board_RTT_reset_N_factory_reset_proc(void)
{
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN

#ifdef FEATURE_CFG_RTT_MODULE_CONTROL
    SEGGER_RTT_printf(0, "\n==TBC_OVER_RTT Testmode==\n");  //start marker is "==TBC_OVER_RTT Testmode==" don't change this
    memset(rtt_rd_bufffer, 0, sizeof(rtt_rd_bufffer));
#endif
    while(1)
    {
        rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN
        if(rtt_rd_size == 2 && ((rtt_rd_bufffer[0] == 'C') && ((rtt_rd_bufffer[1] == 'R') || (rtt_rd_bufffer[1] == 'F'))))
        {
            if((rtt_rd_bufffer[1] == 'R'))
            {               
                cfg_board_reset();  //reset work
            }
            else if(rtt_rd_bufffer[1] == 'F')
            {
                module_parameter_erase_and_reset();  //factory reset work
            }
        }
    }
}

static void cfg_board_testmode_wifi(void)
{
    SEGGER_RTT_printf(0, "====enter wifi rf test mode====\n");
#ifdef CDEV_WIFI_MODULE
    cWifi_enter_rftest_mode();
#endif
    cfg_board_RTT_reset_N_factory_reset_proc();
}

#ifdef CDEV_WIFI_MODULE
static void cfg_board_testmode_wifi_always_on(void)
{
    SEGGER_RTT_printf(0, "====enter wifi always on mode====\n");
#ifdef CDEV_WIFI_MODULE
    cfg_board_gpio_set_default();
    cWifi_power_control(true);
#endif
    cfg_board_RTT_reset_N_factory_reset_proc();
}
#endif

#ifdef CDEV_GPS_MODULE
static const nrf_drv_spi_t m_cfg_board_testmode_gps_Spi = NRF_DRV_SPI_INSTANCE(GPS_SPI_INSTANCE);  /**< SPI instance. */
static volatile bool m_cfg_board_testmode_gps_spi_XferDone; 
static uint8_t *m_cfg_board_testmode_gps_spi_tx_buf;
static uint8_t *m_cfg_board_testmode_gps_spi_rx_buf;
static void cfg_board_testmode_gps_uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_tx_buf_uart_idx < 128)
            {
                m_cfg_board_testmode_tx_buf_uart[m_cfg_board_testmode_tx_buf_uart_idx++] = uart_byte;
            }
//            app_uart_put(uart_byte);  //echo test
            break;

        case APP_UART_COMMUNICATION_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart fifo Err!\n");
            break;

        default:
            break;
    }
}

static void cfg_board_testmode_gps_spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    m_cfg_board_testmode_gps_spi_XferDone = true;
}

static void cfg_board_testmode_gps(void)
{
//    cfg_ble_led_control(1);  //test power led

    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        PIN_DEF_DTM_RX, //RX_PIN_NUMBER,
        PIN_DEF_DTM_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud9600
    };
    nrf_drv_spi_config_t spi_config;
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN
    int i;
    uint32_t tick_cnt = 0, tick_cnt_old_rtt = 0;

    SEGGER_RTT_printf(0, "====enter gps test mode====\n");
    cGps_gpio_init();
    cGps_power_control(1, 1);  //gps power on

    //Since the TBC can not be executed, it uses TBC buffers.
    if((1024 <= CTBC_TX_BUF_SIZE))
    {
        extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
        app_uart_buffers_t buffers;

        memcpy(&spi_config, &m_spi_config_default, sizeof(nrf_drv_spi_config_t));
        spi_config.ss_pin   = PIN_DEF_GPS_SPI_CS;
        spi_config.miso_pin = PIN_DEF_GPS_SPI_MISO;
        spi_config.mosi_pin = PIN_DEF_GPS_SPI_MOSI;
        spi_config.sck_pin  = PIN_DEF_GPS_SPI_SCK;
        spi_config.frequency = NRF_DRV_SPI_FREQ_125K;
        spi_config.mode = NRF_DRV_SPI_MODE_0;
        spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
        APP_ERROR_CHECK(nrf_drv_spi_init(&m_cfg_board_testmode_gps_Spi, &spi_config, cfg_board_testmode_gps_spi_event_handler));
    
        m_cfg_board_testmode_gps_spi_tx_buf = &m_cTBC_tx_buf[0];
        m_cfg_board_testmode_gps_spi_rx_buf = &m_cTBC_tx_buf[128];
        m_cfg_board_testmode_tx_buf_uart = &m_cTBC_tx_buf[256];
        m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[384];

        //ref APP_UART_FIFO_INIT()
        buffers.rx_buf      = &m_cTBC_tx_buf[512];
        buffers.rx_buf_size = 128;
        buffers.tx_buf      = &m_cTBC_tx_buf[640];
        buffers.tx_buf_size = 128;
        err_code = app_uart_init(&comm_params, &buffers, cfg_board_testmode_gps_uart_event_handle, APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(err_code);

        m_cfg_board_testmode_rx_buf_rtt = &m_cTBC_tx_buf[768];
        m_cfg_board_testmode_tx_buf_rtt = &m_cTBC_tx_buf[896];

        while(1)
        {
            tick_cnt++;
            memset(m_cfg_board_testmode_gps_spi_tx_buf, 0xff, 128);
            if(m_cfg_board_testmode_tx_buf_uart_idx > 0)
            {
                CRITICAL_REGION_ENTER();
                memcpy(m_cfg_board_testmode_gps_spi_tx_buf, m_cfg_board_testmode_tx_buf_uart, m_cfg_board_testmode_tx_buf_uart_idx);
                m_cfg_board_testmode_tx_buf_uart_idx = 0;
                CRITICAL_REGION_EXIT();
            }
            else
            {
                rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN
                if(rtt_rd_size > 0)
                {
                    tick_cnt_old_rtt = tick_cnt;
                    for(i=0;i<rtt_rd_size;i++)
                    {
                        m_cfg_board_testmode_rx_buf_rtt[m_cfg_board_testmode_rx_buf_rtt_idx++] = rtt_rd_bufffer[i];
                    }

                }
                if((m_cfg_board_testmode_rx_buf_rtt_idx > 0) && ((tick_cnt_old_rtt + 2) == tick_cnt))
                {
                    memcpy(m_cfg_board_testmode_gps_spi_tx_buf, m_cfg_board_testmode_rx_buf_rtt, m_cfg_board_testmode_rx_buf_rtt_idx);
                    m_cfg_board_testmode_rx_buf_rtt_idx = 0;
                }
            }

            if(((m_cfg_board_testmode_gps_spi_tx_buf[0] == 'C') && ((m_cfg_board_testmode_gps_spi_tx_buf[1] == 'R') || (m_cfg_board_testmode_gps_spi_tx_buf[1] == 'F'))))
            {
                if((m_cfg_board_testmode_gps_spi_tx_buf[1] == 'R'))
                {               
                    cfg_board_reset();  //reset work
                }
                else if(m_cfg_board_testmode_gps_spi_tx_buf[1] == 'F')
                {
                    module_parameter_erase_and_reset();  //factory reset work
                }
            }
            else
            {
                m_cfg_board_testmode_gps_spi_XferDone = false;
                nrf_drv_spi_transfer(&m_cfg_board_testmode_gps_Spi, m_cfg_board_testmode_gps_spi_tx_buf, 128, m_cfg_board_testmode_gps_spi_rx_buf, 128);
                while(!m_cfg_board_testmode_gps_spi_XferDone);

                for(i=0; i<128; i++)
                {
                    if(m_cfg_board_testmode_gps_spi_rx_buf[i] != 0xff)
                    {
                        m_cfg_board_testmode_rx_buf_uart[m_cfg_board_testmode_rx_buf_uart_idx++] = m_cfg_board_testmode_gps_spi_rx_buf[i];
                        m_cfg_board_testmode_tx_buf_rtt[m_cfg_board_testmode_tx_buf_rtt_idx++] = m_cfg_board_testmode_gps_spi_rx_buf[i];
                    }
                }

                if(m_cfg_board_testmode_rx_buf_uart_idx > 0)
                {
                    for(i=0; i<m_cfg_board_testmode_rx_buf_uart_idx; i++)
                    {
                        app_uart_put(m_cfg_board_testmode_rx_buf_uart[i]);
                    }
                    m_cfg_board_testmode_rx_buf_uart_idx = 0;
                }
                

                if(m_cfg_board_testmode_tx_buf_rtt_idx > 0)
                {
                    SEGGER_RTT_Write(0, m_cfg_board_testmode_tx_buf_rtt, m_cfg_board_testmode_tx_buf_rtt_idx);
                    m_cfg_board_testmode_tx_buf_rtt_idx = 0;
                }
            }
            nrf_delay_ms(200);
        }
    }
}
#endif

static void cfg_board_bridge_from_RTT_to_uart_uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_rx_buf_uart_idx < 512)
            {
                m_cfg_board_testmode_rx_buf_uart[m_cfg_board_testmode_rx_buf_uart_idx++] = uart_byte;
            }
//            app_uart_put(uart_byte);  //echo test
            break;

        case APP_UART_COMMUNICATION_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart fifo Err!\n");
            break;

        default:
            break;
    }
}

static void cfg_board_bridge_from_RTT_to_uart(uint32_t baud_rate, uint32_t rx_pin_no, uint32_t tx_pin_no)
{
    uint32_t                     err_code;
    app_uart_comm_params_t comm_params =
    {
        PIN_DEF_DTM_RX, //RX_PIN_NUMBER,
        PIN_DEF_DTM_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN
    int i;
    uint32_t rtt_read_time_out=0;

    SEGGER_RTT_printf(0, "bridge_from_RTT_to_uart baud rate:%08x, rx:%d, tx:%d\n", baud_rate, rx_pin_no, tx_pin_no);
    comm_params.baud_rate = baud_rate;
    comm_params.rx_pin_no = rx_pin_no;
    comm_params.tx_pin_no = tx_pin_no;

    //Since the TBC can not be executed, it uses TBC buffers.
    if((2048 <= CTBC_TX_BUF_SIZE) && (256 <= CTBC_BYPASS_CMD_BUF_SIZE))
    {
        extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
        extern uint8_t m_cTBC_bypasscmd_buf[CTBC_BYPASS_CMD_BUF_SIZE];
        app_uart_buffers_t buffers;

        m_cfg_board_testmode_tx_buf_uart = &m_cTBC_tx_buf[0];
        m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[512];

        //ref APP_UART_FIFO_INIT()
        buffers.rx_buf      = &m_cTBC_bypasscmd_buf[0];
        buffers.rx_buf_size = 128;
        buffers.tx_buf      = &m_cTBC_bypasscmd_buf[128];
        buffers.tx_buf_size = 128;
        err_code = app_uart_init(&comm_params, &buffers, cfg_board_bridge_from_RTT_to_uart_uart_event_handle, APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(err_code);

        m_cfg_board_testmode_rx_buf_rtt = &m_cTBC_tx_buf[1024];
        m_cfg_board_testmode_tx_buf_rtt = &m_cTBC_tx_buf[1536];

        while(1)
        {
            rtt_read_time_out++;
            if(m_cfg_board_testmode_rx_buf_uart_idx > 0)
            {
                CRITICAL_REGION_ENTER();
                memcpy(m_cfg_board_testmode_tx_buf_rtt, m_cfg_board_testmode_rx_buf_uart, m_cfg_board_testmode_rx_buf_uart_idx);
                m_cfg_board_testmode_tx_buf_rtt_idx = m_cfg_board_testmode_rx_buf_uart_idx;
                m_cfg_board_testmode_rx_buf_uart_idx = 0;
                CRITICAL_REGION_EXIT();
            }
            else
            {
                rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN
                if(rtt_rd_size > 0)
                {
                    for(i=0;i<rtt_rd_size;i++)
                    {
                        if(m_cfg_board_testmode_rx_buf_rtt_idx < 512)m_cfg_board_testmode_rx_buf_rtt[m_cfg_board_testmode_rx_buf_rtt_idx++] = rtt_rd_bufffer[i];
                    }
                    rtt_read_time_out = 0;
                }
                if((m_cfg_board_testmode_rx_buf_rtt_idx > 0) && rtt_read_time_out > 40000)  //about 100ms
                {
                    memcpy(m_cfg_board_testmode_tx_buf_uart, m_cfg_board_testmode_rx_buf_rtt, m_cfg_board_testmode_rx_buf_rtt_idx);
                    m_cfg_board_testmode_tx_buf_uart_idx = m_cfg_board_testmode_rx_buf_rtt_idx;
                    m_cfg_board_testmode_rx_buf_rtt_idx = 0;
                }
            }

            if((m_cfg_board_testmode_tx_buf_uart_idx == 2)
                && ((m_cfg_board_testmode_tx_buf_uart[0] == 'C') && ((m_cfg_board_testmode_tx_buf_uart[1] == 'R') || (m_cfg_board_testmode_tx_buf_uart[1] == 'F'))))
            {
                if((m_cfg_board_testmode_tx_buf_uart[1] == 'R'))
                {               
                    cfg_board_reset();  //reset work
                }
                else if(m_cfg_board_testmode_tx_buf_uart[1] == 'F')
                {
                    module_parameter_erase_and_reset();  //factory reset work
                }
            }

            if(m_cfg_board_testmode_tx_buf_rtt_idx > 0)
            {
                SEGGER_RTT_Write(0, m_cfg_board_testmode_tx_buf_rtt, m_cfg_board_testmode_tx_buf_rtt_idx);
                m_cfg_board_testmode_tx_buf_rtt_idx = 0;
            }

            if(m_cfg_board_testmode_tx_buf_uart_idx > 0)
            {
                for(i=0; i<m_cfg_board_testmode_tx_buf_uart_idx; i++)
                {
                    app_uart_put(m_cfg_board_testmode_tx_buf_uart[i]);
                }
                m_cfg_board_testmode_tx_buf_uart_idx = 0;
            }
        }
    }
}

static void cfg_board_uart_bridge_from_external_to_sigfox_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_rx_buf_uart_idx < 256)
            {
                m_cfg_board_testmode_rx_buf_uart[m_cfg_board_testmode_rx_buf_uart_idx++] = uart_byte;
                if(m_cfg_board_testmode_rx_buf_uart_idx == 2 && m_cfg_board_testmode_rx_buf_uart[0]=='C')
                {
                    if(m_cfg_board_testmode_rx_buf_uart[1]=='R')
                    {
                        cfg_board_reset();  //reset work
                    }
                    else if(m_cfg_board_testmode_rx_buf_uart[1]=='F')
                    {
                        module_parameter_erase_and_reset();  //factory reset work
                    }
                }

                if(uart_byte == '\n')
                {
                    m_cfg_bridge_from_uart_to_uart_flag = true;
                }
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "external uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "external uart fifo Err!\n");
            break;

        default:
            break;
    }
}

static void cfg_board_uart_bridge_from_sigfox_to_external_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_rx_buf_uart_idx_2nd < 256)
            {
                m_cfg_board_testmode_rx_buf_uart_2nd[m_cfg_board_testmode_rx_buf_uart_idx_2nd++] = uart_byte;
                if(uart_byte == '\n')
                {
                    m_cfg_testmode_wait_tick = 1;
                }
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "sigfox uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "sigfox uart fifo Err!\n");
            break;

        default:
            break;
    }
}


static void cfg_board_sigfox_bridge_from_uart(uint32_t baud_rate, uint32_t rx_pin_no, uint32_t tx_pin_no)
{
    uint32_t err_code;
    int i;
    bool bridge_from_uart_to_uart_flag_old;
    app_uart_comm_params_t comm_params =
    {
        PIN_DEF_DTM_RX, //RX_PIN_NUMBER,
        PIN_DEF_DTM_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    app_uart_buffers_t buffers;

    app_uart_comm_params_t comm_params_sigfox =
    {
        PIN_DEF_SIGFOX_UART_RX, //RX_PIN_NUMBER,
        PIN_DEF_SIGFOX_UART_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud9600
    };
    app_uart_buffers_t buffers_sigfox;

    
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN

    SEGGER_RTT_printf(0, "sigfox_bridge_from_uart baud rate:%08x, in rx:%d, in tx:%d\n", baud_rate, rx_pin_no, tx_pin_no);
    comm_params.baud_rate = baud_rate;
    comm_params.rx_pin_no = rx_pin_no;
    comm_params.tx_pin_no = tx_pin_no;

    //Since the TBC can not be executed, it uses TBC buffers.
    if((2048 <= CTBC_TX_BUF_SIZE))
    {
        extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
        extern uint8_t m_cTBC_bypasscmd_buf[CTBC_BYPASS_CMD_BUF_SIZE];

        m_cfg_board_testmode_tx_buf_uart = &m_cTBC_tx_buf[0];
        m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[256];
        m_cfg_board_testmode_tx_buf_uart_2nd = &m_cTBC_tx_buf[512];
        m_cfg_board_testmode_rx_buf_uart_2nd = &m_cTBC_tx_buf[768];


        //ref APP_UART_FIFO_INIT()
        buffers.rx_buf      = &m_cTBC_tx_buf[1024];
        buffers.rx_buf_size = 128;
        buffers.tx_buf      = &m_cTBC_tx_buf[1152];
        buffers.tx_buf_size = 128;
        buffers_sigfox.rx_buf      = &m_cTBC_tx_buf[1280];
        buffers_sigfox.rx_buf_size = 128;
        buffers_sigfox.tx_buf      = &m_cTBC_tx_buf[1408];
        buffers_sigfox.tx_buf_size = 128;

        bridge_from_uart_to_uart_flag_old = m_cfg_bridge_from_uart_to_uart_flag = false;
        //connect external
        err_code = app_uart_init(&comm_params, &buffers, cfg_board_uart_bridge_from_external_to_sigfox_event_handle, APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(err_code);
        

        while(1)
        {
            //ref cfg_board_RTT_reset_N_factory_reset_proc for RTT Factroy reset
            rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN 
            if(rtt_rd_size == 2 && ((rtt_rd_bufffer[0] == 'C') && ((rtt_rd_bufffer[1] == 'R') || (rtt_rd_bufffer[1] == 'F'))))
            {
                if((rtt_rd_bufffer[1] == 'R'))
                {               
                    cfg_board_reset();  //reset work
                }
                else if(rtt_rd_bufffer[1] == 'F')
                {
                    module_parameter_erase_and_reset();  //factory reset work
                }
            }

            if(bridge_from_uart_to_uart_flag_old != m_cfg_bridge_from_uart_to_uart_flag)
            {
                app_uart_close();
                if(m_cfg_bridge_from_uart_to_uart_flag)
                {
                    //connect sigfox
                    err_code = app_uart_init(&comm_params_sigfox, &buffers_sigfox, cfg_board_uart_bridge_from_sigfox_to_external_event_handle, APP_IRQ_PRIORITY_LOW);
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    //connect external
                    err_code = app_uart_init(&comm_params, &buffers, cfg_board_uart_bridge_from_external_to_sigfox_event_handle, APP_IRQ_PRIORITY_LOW);
                    APP_ERROR_CHECK(err_code);
                }
                nrf_delay_ms(1);
                if(m_cfg_bridge_from_uart_to_uart_flag)  //external to sigfox (uart connected to sigfox)
                {
                    CRITICAL_REGION_ENTER();
                    memcpy(m_cfg_board_testmode_tx_buf_uart_2nd, m_cfg_board_testmode_rx_buf_uart, m_cfg_board_testmode_rx_buf_uart_idx);
                    m_cfg_board_testmode_tx_buf_uart_idx_2nd = m_cfg_board_testmode_rx_buf_uart_idx;
                    m_cfg_board_testmode_rx_buf_uart_idx = 0;
                    CRITICAL_REGION_EXIT();
                    for(i=0; i<m_cfg_board_testmode_tx_buf_uart_idx_2nd; i++)
                    {
                        app_uart_put(m_cfg_board_testmode_tx_buf_uart_2nd[i]);
                    }
                    m_cfg_board_testmode_tx_buf_uart_idx_2nd = 0;
                    m_cfg_testmode_wait_tick = 0;
                }
                else  //sigfox to external (uart connected to external)
                {
                    CRITICAL_REGION_ENTER();
                    memcpy(m_cfg_board_testmode_tx_buf_uart, m_cfg_board_testmode_rx_buf_uart_2nd, m_cfg_board_testmode_rx_buf_uart_idx_2nd);
                    m_cfg_board_testmode_tx_buf_uart_idx = m_cfg_board_testmode_rx_buf_uart_idx_2nd;
                    m_cfg_board_testmode_rx_buf_uart_idx_2nd = 0;
                    CRITICAL_REGION_EXIT();
                    for(i=0; i<m_cfg_board_testmode_tx_buf_uart_idx; i++)
                    {
                        app_uart_put(m_cfg_board_testmode_tx_buf_uart[i]);
                    }
                    m_cfg_board_testmode_tx_buf_uart_idx = 0;
                }
                bridge_from_uart_to_uart_flag_old = m_cfg_bridge_from_uart_to_uart_flag;
            }
            
            if(m_cfg_bridge_from_uart_to_uart_flag)  //wait sigfox resp
            {
                if(m_cfg_testmode_wait_tick >= 1)
                {
                    if(++m_cfg_testmode_wait_tick > 5)  //timeout
                    {
                        m_cfg_bridge_from_uart_to_uart_flag = false; //wait external input
                        m_cfg_testmode_wait_tick = 0;
                    }
                }
            }
            nrf_delay_ms(100);
        }
    }
}

static void cfg_board_testmode_ble(void)
{
    SEGGER_RTT_printf(0, "====enter ble dtm mode====\n");
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
#else
static void cfg_board_check_wifi_downloadmode(void)
{
    uint32_t pinLvl_DL_EN;
    nrf_gpio_cfg_input(PIN_DEF_WIFI_INT, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(1);

    pinLvl_DL_EN = nrf_gpio_pin_read(PIN_DEF_WIFI_INT);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_INT);
    cPrintLog(CDBG_FCTRL_INFO, "wifi dl pin lvl:%d\n", pinLvl_DL_EN);

    if(pinLvl_DL_EN == 0)
    {
        cfg_board_testmode_gps_wifi_download();
    }
}
#endif

#ifdef FEATURE_CFG_CHECK_NV_BOOT_MODE
static void cfg_board_testmode_wifi_AP_N_ble_on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
//    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
 //           err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
 //           APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            break;

        default:
            break;
    }
}

static void cfg_board_testmode_wifi_AP_N_ble(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;
    uint8_t device_name_buf[32];
    uint32_t device_name_len;

#ifdef CDEV_WIFI_MODULE
    int wifi_result;
    int timeout;
    uint8_t cmd_buf[128];
    unsigned int cmd_buf_idx;
    
    cfg_ble_led_control(true);
    main_examples_prepare();
    //disable wifi info log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));

    cWifi_resource_init();  // Initalize resource for WIFI module 
    cWifi_prepare_start();  // prepare for WIFI module

    wifi_result = cWifi_bypass_req(NULL, NULL);
    if(wifi_result == CWIFI_Result_OK)
    {
        timeout = 5000;
        while(!cWifiState_is_bypass_mode())
        {
            if(--timeout==0)break;  //wait bypassmode
            nrf_delay_ms(1);
        }
        if(timeout > 0)
        {
            cmd_buf_idx = sprintf((char *)cmd_buf, "ATE0");
            cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
            //wait bypass ready
            while(!cWifiState_is_bypass_ready());
            cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
            nrf_delay_ms(500);  //wait for response

            cmd_buf_idx = sprintf((char *)cmd_buf, "AT+CWMODE=2");
            cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
            //wait bypass ready
            while(!cWifiState_is_bypass_ready());
            cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
            nrf_delay_ms(1000);  //wait for response

            //make at command
            cmd_buf_idx = sprintf((char *)cmd_buf, "AT+CWSAP=\"SFMTEST%02x%02x\",\"1234567890\",5,3", m_module_peripheral_ID.wifi_MAC_STA[4], m_module_peripheral_ID.wifi_MAC_STA[5]);
            cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
            while(!cWifiState_is_bypass_ready());
            cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
            nrf_delay_ms(500);  //wait for response
        }
        else
        {
            cPrintLog(CDBG_MAIN_LOG, "Wifi bypassmode timeout!\n");
        }
    }
    else
    {
        // WIFI not available or busy
        cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
    }
#endif

    device_name_len = sprintf((char *)device_name_buf, "SFMTEST%02x%02x", m_module_peripheral_ID.ble_MAC[4], m_module_peripheral_ID.ble_MAC[5]);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                      (const uint8_t *)device_name_buf,
                                      device_name_len);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS) ;
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS);
    gap_conn_params.slave_latency     = 0;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS) ;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(4);
    APP_ERROR_CHECK(err_code);

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = MSEC_TO_UNITS(50, UNIT_0_625_MS);
    options.ble_adv_fast_timeout = 0; //APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, cfg_board_testmode_wifi_AP_N_ble_on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);

    ble_advertising_start(BLE_ADV_MODE_FAST);
}

static void cfg_board_check_bootmode(void)
{
    int bootmode;

    if(module_parameter_get_bootmode(&bootmode))  //use to sfm_boot_mode START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC
    {
        switch(bootmode)
        {
            case 1:
                cTBC_setting_erase_wait_for_testmode(3);
#ifdef CDEV_WIFI_MODULE
                cfg_board_testmode_wifi();
#endif
                break;
            case 2:
                cTBC_setting_erase_wait_for_testmode(3);
#ifdef CDEV_WIFI_MODULE
                cfg_board_testmode_wifi_always_on();
#endif
                break;

            case 3:
                cTBC_setting_erase_wait_for_testmode(3);
                cfg_board_testmode_ble();
                break;

            case 4:
                cTBC_setting_erase_wait_for_testmode(3);
#ifdef CDEV_GPS_MODULE
                cfg_board_testmode_gps();
#else
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;
            case 5:
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter wifi rf test bridge_from_RTT_to_uart mode====\n");
#ifdef CDEV_WIFI_MODULE
                cWifi_enter_rftest_mode();
#endif
                cfg_board_bridge_from_RTT_to_uart(UART_BAUDRATE_BAUDRATE_Baud115200, PIN_DEF_DTM_RX, PIN_DEF_DTM_TX);
                break;

            case 6:
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter sigfox uart over RTT mode====\n");
                
                //power enable pin control   
                cfg_board_common_power_control(module_comm_pwr_sigfox, true);
                nrf_delay_ms(10);
                nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
                nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
                nrf_delay_ms(10);  //spec is 4ms
                nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
                nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
                nrf_delay_ms(10);
                nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 1);
                nrf_delay_ms(1000);
                
                cfg_board_bridge_from_RTT_to_uart(UART_BAUDRATE_BAUDRATE_Baud9600, PIN_DEF_SIGFOX_UART_RX, PIN_DEF_SIGFOX_UART_TX);
                break;

            case 7:
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter sigfox over Uart====\n");
                
                //power enable pin control   
                cfg_board_common_power_control(module_comm_pwr_sigfox, true);
                nrf_delay_ms(10);
                nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
                nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
                nrf_delay_ms(10);  //spec is 4ms
                nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
                nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
                nrf_delay_ms(10);
                nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 1);
                nrf_delay_ms(1000);

                cfg_board_sigfox_bridge_from_uart(UART_BAUDRATE_BAUDRATE_Baud9600, PIN_DEF_DTM_RX, PIN_DEF_DTM_TX);
                break;

            case 8:  //WIFI AP(SFMTEST0000) and BLE BEACON(SFMTEST0000)
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter WIFI AP(SFMTEST0000) and BLE BEACON(SFMTEST0000)====\n");
                cfg_board_testmode_wifi_AP_N_ble();
                cfg_board_RTT_reset_N_factory_reset_proc();
                break;

            default:
                break;
        }
    }
}
#endif

void cfg_board_early_init(void)
{
    cfg_board_check_reset_reason();
#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
    cfg_board_check_bootstrap_pin();
#else
    cfg_board_check_wifi_downloadmode();
#endif
#ifdef FEATURE_CFG_CHECK_NV_BOOT_MODE
    cfg_board_check_bootmode();
#endif
    cfg_board_gpio_set_default();
    cfg_board_check_bootloader();
}

void cfg_board_init(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "%s\n", __func__);
    cfg_board_resource_init();

    cfg_board_common_power_control(module_comm_pwr_pon_init, true);
    nrf_delay_ms(2);

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

