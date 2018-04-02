#ifndef __CFG_BOARD_DEF_H__
#define __CFG_BOARD_DEF_H__
/******************************************************
module info
*******************************************************/
#define CDEV_MODEL_NAME "SFM20R"   //MODEL NAME SIZE IS 6BYTE
/**********************
module defines (CDEV_)
SFM20R : MODEL_NAME to "SFM20R"
         MODULE_TYPE to CDEV_MODULE_SFM20R
SFM40R : MODEL_NAME to "SFM40R"
         MODULE_TYPE to CDEV_MODULE_SFM40R
SFM50R : MODEL_NAME to "SFM50R"
         MODULE_TYPE to CDEV_MODULE_SFM50R
SFM60R : MODEL_NAME to "SFM60R"
         MODULE_TYPE to CDEV_MODULE_SFM60R
***********************/
#define CDEV_SW_VER_MAJOR "2"       // 1byte
#define CDEV_SW_VER_MINOR "04"      // 2byte
#define CDEV_FS_VER 0x0021          //module_parameter_t version
#define CDEV_BL_VER "1"             // 1byte
#define CDEV_BL_SETTING_VER "1"     // 1byte ref NRF_DFU_SETTINGS_VERSION predeine
#define CDEV_HW_VER "52"            //NRF_DFU_HW_VERSION NRF52

/******************************************************
wisol feature
*******************************************************/
//#define FEATURE_WISOL_DEVICE //move to predefine
//#define FEATURE_WISOL_BOOTLOADER //move to predefine
//#define FEATURE_WISOL_APP //move to predefine
#define WISOL_BOOTLOADER_RESERVED_END_ADDR 0x78000   //depaned on FEATURE_WISOL_DEVICE (predefined)

/******************************************************
senario feature
*******************************************************/
#define MODULE_SCENARIO_ASSET_TRACKER           (0)
#define MODULE_SCENARIO_MWC_DEMO                (1)
#define MODULE_SCENARIO_IHERE_MINI              (2)
#define MODULE_SCENARIO_MODE_MAX                MODULE_SCENARIO_IHERE_MINI
#define MODULE_DEFAULT_SCENARIO_TYPE            MODULE_SCENARIO_ASSET_TRACKER

/******************************************************
module feature
*******************************************************/
#define CDEV_MODULE_SFM20R                      (1)
#define CDEV_MODULE_SFM40R                      (2)
#define CDEV_MODULE_SFM50R                      (3)
#define CDEV_MODULE_SFM60R                      (4)
    
#define CDEV_MODULE_TYPE                        CDEV_MODULE_SFM20R

/******************************************************
board feature
*******************************************************/
#define CDEV_BOARD_EVB                         (1)
#define CDEV_BOARD_IHERE                       (2)
#define CDEV_BOARD_IHERE_MINI                  (3)
#define CDEV_BOARD_IHEREV2                     (4)
#define CDEV_BOARD_M3                          (5)

#define CDEV_BOARD_TYPE                        CDEV_BOARD_EVB  //REPLACE_DEVICE_DEFINE_HERE
/******************************************************
module feature
*******************************************************/
#define CDEV_WIFI_MODULE  //ESP8285
#define CDEV_GPS_MODULE   //UBX-G8020
//#define FEATURE_ONSEMI_IHERE_DEMO_TEST // 2018.03.27

#define CDEV_BLE_ADVERTISING_ENABLE
//#define CDEV_SIGFOX_RCZ24
/******************************************************
external sensor feature
*******************************************************/
#define CDEV_MAGNETIC_SENSOR  //GPIO MAGNETIC SENSOR (USE GPIO INTERRUPT), and wkup key

/******************************************************/

/******************************************************
funcion feature
*******************************************************/
#if (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)
#define FEATURE_CFG_CHECK_BOOTSTRAP_PIN  //support boot strap mode
#endif
#define FEATURE_CFG_CHECK_NV_BOOT_MODE  //support bootmode (m_module_parameter)
#define FEATURE_CFG_DEBUG_PRINT_OUT //debug print out
#define FEATURE_CFG_DEBUG_OUT_TO_TBC //depend on FEATURE_CFG_DEBUG_PRINT_OUT
#define FEATURE_CFG_BYPASS_CONTROL
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2) || (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
#define FEATURE_CFG_ACC_REPORT
#endif
#define FEATURE_CFG_RTT_MODULE_CONTROL //RTT over TBC(twis board control), Test mode control via RTT

#ifdef CDEV_GPS_MODULE
//#define CDEV_UBLOX_GPS_MODULE  //  not used
//#define CDEV_CSR_GPS_MODULE // not used - CSR gps
#if (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
#define PIN_DEF_GPS_BKUP_CTRL_WITH_PULLUP PIN_DEF_TWIS_BOARD_CTRL_SDA  //V_BCKP_GPS contorl by gpio with pulledup  //GPS_BKUP_CTRL
#endif
#endif
#define CDEV_NUS_MODULE
#define CDEV_RTC2_DATE_TIME_CLOCK
/******************************************************/

/******************************************************
test feature
*******************************************************/
//#define SIGFOX_SNEK_TEST
//#define TEST_SIGFOX_CURRENT_CONSUMPTION  // Just current consumption test mode(All module mode)

/******************************************************/

#ifndef FEATURE_WISOL_BOOTLOADER
#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         12                                /**< Size of timer operation queues. */
#endif
#define APP_MAIN_SCHEDULE_MS            200
#define APP_MAIN_SCHEDULE_HZ            (1000/APP_MAIN_SCHEDULE_MS)  //for use main_schedule_tick

//SPI INSTANCE
#define WIFI_SPI_INSTANCE  2  /**< SPI instance index. */
#define GPS_SPI_INSTANCE   2  /**< SPI instance index. */

//Gsensor
#define GSEN_SPI_INSTANCE  0
#define GSEN_TWI_INSTANCE  0

//I2C INSTANCE
#define ACCELEROMETER_TWI_INSTANCE      0
#define TBC_TWIS_INSTANCE               1  /**< I2C instance index for  twis board control ref TWIS_ENABLED TWIS1_ENABLED*/

#define PIN_DEF_2ND_POW_EN              26
#define PIN_DEF_WIFI_INT                4  //for download mode DL_EN/INT_WIFI

#define PIN_DEF_SIGFOX_PWR_EN     24
#define PIN_DEF_SIGFOX_RESET      23
#define PIN_DEF_SIGFOX_UART_TX    25
#define PIN_DEF_SIGFOX_UART_RX    27


#ifdef CDEV_WIFI_MODULE
#define PIN_DEF_WIFI_SPI_MISO   8
#define PIN_DEF_WIFI_SPI_MOSI   6
#define PIN_DEF_WIFI_SPI_CLK    7
#define PIN_DEF_WIFI_SPI_CS     5
#ifndef SPI_IRQ_PRIORITY
#define SPI_IRQ_PRIORITY 6
#endif
#define PIN_DEF_WIFI_PWR_EN     13
#define PIN_DEF_WIFI_RESET      17
#endif

#ifdef CDEV_GPS_MODULE
#define PIN_DEF_GPS_SPI_MISO   28
#define PIN_DEF_GPS_SPI_MOSI   29
#define PIN_DEF_GPS_SPI_SCK    30
#define PIN_DEF_GPS_SPI_CS     31
#if 0
#define PIN_DEF_GPS_SPI_MISO   28
#define PIN_DEF_GPS_SPI_MOSI   31
#define PIN_DEF_GPS_SPI_SCK    30
#define PIN_DEF_GPS_SPI_CS     29
#endif

#define PIN_DEF_GPS_PWR_EN     19
#define PIN_DEF_GPS_RESET      22
#endif

#define PIN_DEF_TWIS_BOARD_CTRL_SCL   (12)  //I2c0_SCL_DBG
#define PIN_DEF_TWIS_BOARD_CTRL_SDA   (14)  //I2c0_SDA_DBG

#define PIN_DEF_ACCELEROMETER_DEVICE_ID      (0x18)  //check SDO pin (to GND:0x18, to VDD:0x19)

#define PIN_DEF_ACC_TWIS_SCL   15
#define PIN_DEF_ACC_TWIS_SDA   16
#define PIN_DEF_ACC_INT1       11

#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2) || (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
#define PIN_DEF_MAGNETIC_SIGNAL       2
#else
#define PIN_DEF_MAGNETIC_SIGNAL       3
#endif

#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI)
#define PIN_DEF_CHARGING_SIGNAL       PIN_DEF_MAGNETIC_SIGNAL
#endif

#define PIN_DEF_WKUP          20

#define PIN_DEF_BLE_LED_EN    18

#define PIN_DEF_AIN0    2
#define PIN_DEF_AIN1    3

#if (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)
#define PIN_DEF_DTM_RX    PIN_DEF_AIN0 //used source_direct_test_mode
#define PIN_DEF_DTM_TX    PIN_DEF_AIN1 //used source_direct_test_mode
#else
#define PIN_DEF_DTM_RX    PIN_DEF_TWIS_BOARD_CTRL_SCL //used source_direct_test_mode
#define PIN_DEF_DTM_TX    PIN_DEF_TWIS_BOARD_CTRL_SDA //used source_direct_test_mode
#endif
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)|| (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
#define PIN_DEF_BATTERY_ADC_INPUT NRF_SAADC_INPUT_AIN1
#else
#define PIN_DEF_BATTERY_ADC_INPUT NRF_SAADC_INPUT_AIN0
#endif

#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
#define PIN_DEF_BOOTSTRAP_CONFIG0  PIN_DEF_BLE_LED_EN         //STATE0/P0.18
#define PIN_DEF_BOOTSTRAP_CONFIG1  PIN_DEF_WKUP                //WKUP/P0.20
#endif

#if (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
#define PIN_DEF_BUTTON PIN_DEF_TWIS_BOARD_CTRL_SCL
#endif

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC_250_PPM      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM}

//modify define
#if (CDEV_MODULE_TYPE == CDEV_MODULE_SFM40R)
#undef CDEV_GPS_MODULE
#elif (CDEV_MODULE_TYPE == CDEV_MODULE_SFM50R)
#undef CDEV_GPS_MODULE
#undef CDEV_WIFI_MODULE
#elif (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)
#undef CDEV_WIFI_MODULE
//#undef CDEV_BLE_ADVERTISING_ENABLE
#endif

#endif // __WBOARD_CONFIG_DEF__
