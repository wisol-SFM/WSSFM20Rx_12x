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
 * @brief control board device control.
 *
 * This file contains the control wifi modle.
 */


#ifndef __CFG_BOARD_H__
#define __CFG_BOARD_H__
#include "cfg_board_def.h"
#include "nrf_drv_spi.h"
#include "cfg_twis_board_control.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MODULE_PARAMETER_MAGIC_TOP          (0xA55A0000 | CDEV_FS_VER)  //fs version
#define MODULE_PARAMETER_MAGIC_BOTTOM       (0xA55AABCD)


#define MAIN_IDLE_TIME_DEFAULT          600
#define BEACON_INTERVAL_TIME_DEFAULT    1000
#define NUS_INTERVAL_TIME_DEFAULT       3    

#define START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC    0    //use to sfm_boot_mode [0:normal, 1:wifi rf test, 2:wifi always on, 3:ble test, 4:gps test mode, 5:wifi rf test uart bridge, 6:sigfox uart over RTT, etc...]
#define START_WAIT_TIME_FOR_CTRL_MODE_CHANGE_SEC        0    //use to sigfox dl enable

#define CWIFI_SEC_RETRY_TIMEOUT_SEC_DEFAULT 10   //0 is wifi off
#define CWIFI_SEC_RETRY_TIMEOUT_SEC_MIN 1  //Scan once
#define CWIFI_SEC_RETRY_TIMEOUT_SEC_MAX 60
#define CGPS_ACQUIRE_TRACKING_TIME_SEC 60 // 60 sec

#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
#define CGPS_CN0_CURRENT_SAVETIME_ENABLE true
#else
#define CGPS_CN0_CURRENT_SAVETIME_ENABLE false
#endif

#define MAIN_SCENARIO_MODE_DEFAULT MODULE_DEFAULT_SCENARIO_TYPE
#define MAIN_BOOT_NFC_UNLOCK_DEFAULT true
#define MAIN_FOTA_ENABLE_DEFAULT true
#define MAIN_MAGNETIC_GPIO_ENABLE_DEFAULT true
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)

#define MAIN_WKUP_GPIO_ENABLE_DEFAULT 2   //LED CONTROL
#undef CWIFI_SEC_RETRY_TIMEOUT_SEC_DEFAULT
#define CWIFI_SEC_RETRY_TIMEOUT_SEC_DEFAULT CWIFI_SEC_RETRY_TIMEOUT_SEC_MIN

#elif (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI)

#undef MAIN_WKUP_GPIO_ENABLE_DEFAULT
#define MAIN_WKUP_GPIO_ENABLE_DEFAULT 2   //LED CONTROL

#undef CWIFI_SEC_RETRY_TIMEOUT_SEC_DEFAULT
#define CWIFI_SEC_RETRY_TIMEOUT_SEC_DEFAULT CWIFI_SEC_RETRY_TIMEOUT_SEC_MIN

#undef MODULE_DEFAULT_SCENARIO_TYPE
#define MODULE_DEFAULT_SCENARIO_TYPE            MODULE_SCENARIO_IHERE_MINI

#undef MAIN_BOOT_NFC_UNLOCK_DEFAULT
#define MAIN_BOOT_NFC_UNLOCK_DEFAULT false

#undef MAIN_MAGNETIC_GPIO_ENABLE_DEFAULT
#define MAIN_MAGNETIC_GPIO_ENABLE_DEFAULT false

#else

#define MAIN_WKUP_GPIO_ENABLE_DEFAULT 1

#endif

#ifdef TEST_FEATURE_CDEV_WIFI_TEST_MODE
#define MAIN_WIFI_TESTMODE_ENABLE_DEFAULT true
#else
#define MAIN_WIFI_TESTMODE_ENABLE_DEFAULT false
#endif

#define MAIN_SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT false

//modify define
#if (MODULE_DEFAULT_SCENARIO_TYPE == MODULE_SCENARIO_MWC_DEMO)  //mwc demo
#undef MAIN_SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT
#undef MAIN_BOOT_NFC_UNLOCK_DEFAULT
#undef MAIN_IDLE_TIME_DEFAULT

#define MAIN_SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT false
#define MAIN_BOOT_NFC_UNLOCK_DEFAULT false
#define MAIN_IDLE_TIME_DEFAULT (60 * 60 * 1)  // 1 hour
#endif
/**
 * @brief Enumerator used for setting items
 */
typedef enum
{
    module_parameter_item_idle_time                                     =  0,  //MAIN_IDLE_TIME_DEFAULT                          60 ~ 604800(sec)
    module_parameter_item_beacon_interval                               =  1,  //BEACON_INTERVAL_TIME_DEFAULT                    20 ~ 10240(ms) 
    module_parameter_item_wifi_scan_retry_time_sec                      =  2,  //CWIFI_SEC_RETRY_TIMEOUT_SEC_DEFAULT             1 ~ 60(sec)
    module_parameter_item_start_wait_time_for_board_control_attach_sec  =  3,  //START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC    1 ~ 10(sec)  //use to sfm_boot_mode START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC
    module_parameter_item_start_wait_time_for_ctrl_mode_change_sec      =  4,  //START_WAIT_TIME_FOR_CTRL_MODE_CHANGE_SEC        1 ~ 30(sec)  //use to sigfox dl enable
    module_parameter_item_gps_tracking_time_sec                         =  5,  //CGPS_ACQUIRE_TRACKING_TIME_SEC                  30 ~ 300(sec)
    module_parameter_item_boot_nfc_unlock                               =  6,  //MAIN_BOOT_NFC_UNLOCK_DEFAULT                    0 ~ 1
    module_parameter_item_fota_enable                                   =  7,  //MAIN_FOTA_ENABLE_DEFAULT                        0 ~ 1
    module_parameter_item_scenario_mode                                 =  8,  //MAIN_SCENARIO_MODE_DEFAULT                      0 ~ 1
    module_parameter_item_magnetic_gpio_enable                          =  9,  //MAIN_MAGNETIC_GPIO_ENABLE_DEFAULT               0 ~ 1
    module_parameter_item_wkup_gpio_enable                              = 10,  //MAIN_WKUP_GPIO_ENABLE_DEFAULT                   0 ~ 2
    module_parameter_item_wifi_testmode_enable                          = 11,  //MAIN_WIFI_TESTMODE_ENABLE_DEFAULT               0 ~ 1  //use to disable_battery_power_down
    module_parameter_item_snek_testmode_enable                          = 12,  //MAIN_SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT        0 ~ 1
    module_parameter_item_gps_cn0_current_savetime_enable               = 13,  //CGPS_CN0_CURRENT_SAVETIME_ENABLE                0 ~ 1
    module_parameter_item_max
}module_parameter_item_e;

typedef enum
{
    module_comm_pwr_pon_init,
    module_comm_pwr_common,
    module_comm_pwr_sigfox,
    module_comm_pwr_gps,
    module_comm_pwr_wifi,
    module_comm_pwr_max
}module_comm_pwr_resource_e;


/**
 * @brief setting items
 */
typedef struct
{
    uint32_t    magic_top;
    uint32_t    idle_time;                                      //module_parameter_item_idle_time
    uint32_t    beacon_interval;                                //module_parameter_item_beacon_interval
    uint32_t    wifi_scan_retry_time_sec;                       //module_parameter_item_wifi_scan_retry_time_sec
    uint32_t    start_wait_time_for_board_control_attach_sec;   //module_parameter_item_start_wait_time_for_board_control_attach_sec //use to sfm_boot_mode START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC
    uint32_t    start_wait_time_for_ctrl_mode_change_sec;       //module_parameter_item_start_wait_time_for_ctrl_mode_change_sec  //use to sigfox dl enable
    uint32_t    gps_acquire_tracking_time_sec;                  //module_parameter_item_gps_tracking_time_sec
    bool        boot_nfc_unlock;                                //module_parameter_item_boot_nfc_unlock
    bool        fota_enable;                                    //module_parameter_item_fota_enable
    uint16_t /*MODULE_DEFAULT_SCENARIO_TYPE*/ scenario_mode;    //module_parameter_item_scenario_mode  //MODULE_DEFAULT_SCENARIO_TYPE
    bool        magnetic_gpio_enable;                           //module_parameter_item_magnetic_gpio_enable
    char        wkup_gpio_enable;                               //module_parameter_item_wkup_gpio_enable  //MAIN_WKUP_GPIO_ENABLE_DEFAULT
    bool        wifi_testmode_enable;                           //module_parameter_item_wifi_testmode_enable  //use to disable_battery_power_down
    bool        sigfox_snek_testmode_enable;                    //module_parameter_item_snek_testmode_enable
    bool        cgps_cn0_current_savetime_enable;               //module_parameter_item_gps_cn0_current_savetime_enable

    //setting value here
    uint32_t    magic_bottom;
    uint32_t    guard_area_align4;
    
    //id cache value here
    uint8_t     wifi_MAC_STA[6];
    uint8_t     sigfox_device_ID[4];
    uint8_t     sigfox_pac_code[8];
    uint8_t     board_ID;
    int32_t     guard_area_align4_2;
}module_parameter_t;

typedef struct
{
    uint8_t     module;
    uint8_t     gps_data[8];
    uint8_t     gps_cn0[1];    
    uint8_t     temperature_data[2];
    uint8_t     wifi_data[12];
    int8_t      wifi_rssi[2];
    uint8_t     battery_volt[1];
    uint8_t     acc_x[2];
    uint8_t     acc_y[2];
    uint8_t     acc_z[2];
    uint8_t     magnet_event;
    uint8_t     accellometer_event;
    uint8_t     ap_key[8];
	uint8_t     report_count[1];
	uint8_t     wifi_scan_time[1];
	uint8_t     gps_tracking_time[1];
}nus_service_parameter_t;

typedef struct
{
    bool        sigfixdata_gps_flag;
    uint8_t     sigfixdata_gps[12];  //sigfox send request data : N/S Indicator, latitude, E/W indicator, longitude Status, Temperature  ...
    bool        sigfixdata_wifi_flag;
    uint8_t     sigfixdata_wifi[12];  //sigfox send request data : couple of wifi rssid
    bool        gps_status;   // 1 fix, 0 not available 
    uint8_t     gps_data[8];  //latitude 4byte, Longtitude
    uint8_t     temperature_data[2];
    uint8_t     magnet_status;   //0 open, 1 close
}module_peripheral_data_t;

typedef struct
{
    uint8_t     ble_MAC[6];
    uint8_t     UUID[16];
    uint8_t     wifi_MAC_STA[6];
    uint8_t     sigfox_device_ID[4];
    uint8_t     sigfox_pac_code[8];
}module_peripheral_ID_t;

#ifdef CDEV_RTC2_DATE_TIME_CLOCK
typedef struct
{
    uint16_t year;
    uint8_t  month;  //first month is 1 : 1 to 12 (not 0 to 11)
    uint8_t  day;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
    uint8_t  dayOfWeek;
} cfg_date_time_t;
#endif

extern const nrf_drv_spi_config_t m_spi_config_default;
extern module_parameter_t m_module_parameter;
extern bool m_module_parameter_update_req;
extern const char m_cfg_sw_ver[4];
extern const char m_cfg_model_name[];
extern const char m_cfg_board_type;
extern const char m_cfg_build_date[];
extern const char m_cfg_build_time[];

extern volatile bool main_wakeup_interrupt;
extern bool m_cfg_sw_reset_detected;
extern bool m_cfg_debug_interface_wake_up_detected;
extern bool m_cfg_available_bootloader_detected;
extern bool m_cfg_NFC_wake_up_detected;
extern bool m_cfg_GPIO_wake_up_detected;
extern bool m_cfg_i2c_master_init_flag;

#ifdef CDEV_NUS_MODULE
extern nus_service_parameter_t m_nus_service_parameter;
#endif
extern module_peripheral_data_t m_module_peripheral_data;
extern module_peripheral_ID_t m_module_peripheral_ID;

//util function//////////////////////////////////////////////////////////////////////////
/**
 * @brief       Function for binary to hexadecimal
 *
 * @param[in]   input binary data
 * @param[in]   input binary data size
 * @param[out]  hexadecimal data
 *
 * @return      void
 */
void cfg_bin_2_hexadecimal(const uint8_t *pBin, int binSize, char *pHexadecimal);

/**
 * @brief       Function for hexadecimal (big endian) to integer
 *
 * @param[in]   input hexadecimal data
 * @param[in]   input hexadecimal size (max 8)
 *
 * @return      integer value
 */
uint32_t cfg_hexadecimal_2_uint_val_big(const char *pHexadecimal, int len);

/**
 * @brief       Function for binary to hexadecimal
 *
 * @param[in]   input pHexadecimal data
 * @param[in]   input HexadeccoimalByteCnt data size (byte count)
 * @param[out]  binary data
 *
 * @return      void
 */
void cfg_hexadecimal_2_bin(const char *pHexadecimal, int HexadeccoimalByteCnt, uint8_t *pBin);

/**
 * @brief       Function for string to interger
 *
 * @param[in]   input string
 *
 * @return      interger value
 */
int cfg_atoi(const char *str);

#ifdef CDEV_RTC2_DATE_TIME_CLOCK
void RTC2_date_time_clock_init_N_start(void);  //date_time
uint32_t date_time_get_timestamp(void);  //date_time
void date_time_set_timestamp(uint32_t timestamp);
void date_time_get_current_time(cfg_date_time_t *tm);
#endif

/**@brief Function for initialising I2C .
 *
 *                         
 */
void cfg_i2c_master_init(void);

/**@brief Function for uninitialising I2C.
 *
 *                         
 */
void cfg_i2c_master_uninit(void);
/////////////////////////////////////////////////////////////////////////////////////////
unsigned int main_get_param_val(module_parameter_item_e item);
void main_set_param_val(module_parameter_item_e item, unsigned int val);
bool main_schedule_state_is_idle(void);
bool module_parameter_erase_and_reset(void);
bool module_parameter_get_bootmode(int *bootmode);

/////////////////////////////////////////////////////////////////////////////////////////
void cfg_board_early_init(void);
void cfg_board_init(void);
void cfg_board_common_power_control(module_comm_pwr_resource_e resource, bool bOn);
bool cfg_get_ble_led_status(void);
bool cfg_ble_led_control(bool bOn);
bool cfg_is_3colorled_contorl(void);
void cfg_wkup_output_control(bool bOn);  // ihere 3color led reference m_module_parameter.wkup_gpio_enable==2
void cfg_board_reset(void);
void cfg_board_check_reset_reason(void);
void cfg_board_gpio_set_default(void);
void cfg_board_gpio_set_default_gps(void);

#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
