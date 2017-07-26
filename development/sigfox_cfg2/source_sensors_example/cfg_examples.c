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
 * @brief  Example Application cfg_example.c file.
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

#include "cfg_dbg_log.h"
#include "cfg_board_def.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_board.h"
#include "cfg_external_sense_gpio.h"
#include "cfg_sigfox_module.h"
#include "cfg_app_main.h"
#include "cfg_gps_module.h"

#define CFG_EXAMPLES_TYPE_NONE              0
#define CFG_EXAMPLES_SCAN_TWO_BSSID         1
#define CFG_EXAMPLES_SCAN_FILTERED          2
#define CFG_EXAMPLES_MAGNET_SENSOR          3
#define CFG_EXAMPLES_WKUP_GPIO              4
#define CFG_EXAMPLES_SIFOX                  5
#define CFG_EXAMPLES_GPS                    6
#define CFG_EXAMPLES_WIFI_BYPASS            7

#define CFG_EXAMPLES_TYPE_DEF CFG_EXAMPLES_TYPE_NONE

#if (CFG_EXAMPLES_TYPE_DEF != CFG_EXAMPLES_TYPE_NONE)
#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_SCAN_TWO_BSSID)
void cfg_examples_wifi_scan_two_bssid(void)
{
    int wifi_result;
    uint8_t *bssidBuf;

    //disable wifi info log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));

    cWifi_resource_init();  // Initalize resource for WIFI module 
    cWifi_prepare_start();  // prepare for WIFI module

    wifi_result = cWifi_ap_scan_req();
    if(wifi_result == CWIFI_Result_OK)
    {
        cPrintLog(CDBG_MAIN_LOG, "WIFI MODULE started!\n");
        while(!(!cWifi_is_scan_state() && !cWifi_bus_busy_check()));  //wait scan
        wifi_result = cWifi_get_BSSIDs_bufPtr(&bssidBuf);
        if(wifi_result == CWIFI_Result_OK)
        {
            //scan success bssidBuf[0]~[5]:mac 1, bssidBuf[6]~[11]:mac 2 -> sorted by RSSI
            cPrintLog(CDBG_MAIN_LOG, "WIFI MODULE cWifi_ap_scan_req ok:");
            cDataDumpPrintOut(CDBG_MAIN_LOG, bssidBuf, (CWIFI_BSSID_CNT*CWIFI_BSSID_SIZE));
        }
        else if(wifi_result == CWIFI_Result_NoData)
        {
            //No AP found
            cPrintLog(CDBG_MAIN_LOG, "WIFI MODULE NoData!\n");
        }
        else
        {
            //scan fail
            cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
        }
    }
    else
    {
        // WIFI not available or busy
        cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
    }
}
#endif


#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_SCAN_FILTERED)
void cfg_examples_wifi_scan_filtered(void)
{
    int wifi_result;
    uint8_t *bssidBuf;
    const char *prefixSSID = "OFFICE";

    //disable wifi info log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));
    cWifi_resource_init();  // Initalize resource for WIFI module 
    cWifi_prepare_start();  // prepare for WIFI module

    wifi_result = cWifi_ap_get_available_first_BSSID(prefixSSID);
    if(wifi_result == CWIFI_Result_OK)
    {
        cPrintLog(CDBG_MAIN_LOG, "WIFI MODULE started! prefix SSID:%s\n", prefixSSID);
        while(!(!cWifi_is_scan_state() && !cWifi_bus_busy_check()));  //wait scan
        wifi_result = cWifi_get_BSSIDs_bufPtr(&bssidBuf);
        if(wifi_result == CWIFI_Result_OK)
        {
            //scan success bssidBuf[0]~[5]:mac 1
            cPrintLog(CDBG_MAIN_LOG, "WIFI MODULE cWifi_ap_get_available_first_BSSID ok:");
            cDataDumpPrintOut(CDBG_MAIN_LOG, bssidBuf, (CWIFI_BSSID_CNT*CWIFI_BSSID_SIZE));
        }
        else if(wifi_result == CWIFI_Result_NoData)
        {
            //No AP found
            cPrintLog(CDBG_MAIN_LOG, "WIFI MODULE NoData!\n");
        }
        else
        {
            //scan fail
            cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
        }
    }
    else
    {
        // WIFI not available or busy
        cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
    }

}
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_MAGNET_SENSOR)
void cfg_examples_magnet_sensor(void)
{
    cfg_magnetic_sensor_init(NULL);
}
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_WKUP_GPIO)
void cfg_examples_wkup_gpio(void)
{
    cfg_wkup_gpio_init(NULL);
}
#endif


#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_SIFOX)
void cfg_examples_sigfox(void)
{

    // user data
    uint8_t test_data[SIGFOX_SEND_PAYLOAD_SIZE];
    uint8_t *p_down_link_data;
    uint32_t down_link_data_size;

    // sending buffer
    extern uint8_t frame_data[(SIGFOX_SEND_PAYLOAD_SIZE*2)+1];  //for hexadecimal

    sprintf((char*)test_data,"AABBCCDDEEFF");

    // create sigfox timer instance
    cfg_sigfox_timer_create();

    // initialize and set initial state ins sigfox state
    sigfox_set_state(SETUP_S);

    //set RCZ
    sigfox_set_rcz(RCZ_1);

    // set flag to decide to receive downlink
    cfg_sigfox_downlink_on_off(true);

    // copy user data to sending buffer
    cfg_bin_2_hexadecimal(test_data, SIGFOX_SEND_PAYLOAD_SIZE, (char *)frame_data);

    // start sigfox module timer
    cfg_sigfox_timers_start();

    // check whether sigfox module finishes
    while(!sigfox_check_exit_excute());

    // stop sigfox module timer
    cfg_sigfox_timers_stop();

    p_down_link_data = cfg_sigfox_get_downlink_ptr(&down_link_data_size);
    cPrintLog(CDBG_MAIN_LOG, "%s %d SIGFOX downlink data:%s, size:%d\n", __func__, __LINE__, p_down_link_data, down_link_data_size);
}
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_GPS)
void cfg_examples_gps(void)
{
    int get_nmea_result = 0;
    uint8_t *nmea_Buf;

// Initalize resource for GPS module(initializing gpio of gps)
    cGps_resource_init();

// prepare for GPS module(gps power control)
    cGps_prepare_start();

// available status check (GPS available check)
    if(cGps_status_available() == CGPS_Result_OK )
    {
        cPrintLog(CDBG_MAIN_LOG, "GPS MODULE started!\n");
// nmea data request of gps acquire
        cGps_nmea_acquire_request();  

// gps position fixed check
        while(!(cGps_nmea_position_fix_check()));

// get data of gps nmea information        
        get_nmea_result = cGps_nmea_get_bufPtr(&nmea_Buf);
        if(get_nmea_result == CGPS_Result_OK)
        {
// success of get gps data
            cPrintLog(CDBG_MAIN_LOG, "GPS Module NMEA DATA:");
            cDataDumpPrintOut(CDBG_MAIN_LOG, nmea_Buf, GPS_SEND_PAYLOAD_SIZE);
        }
        else if(get_nmea_result == CGPS_Result_NoData)
        {
// no gps data
            cPrintLog(CDBG_MAIN_LOG, "GPS Module NoData!\n");
        }
        else
        {
// not available GPS
            cPrintLog(CDBG_MAIN_LOG, "Not Available GPS Module!\n");
        }
    }
    else
    {
// not available GPS
        cPrintLog(CDBG_MAIN_LOG, "Not Available GPS Module!\n");
    }
}
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_WIFI_BYPASS)
void cWifi_bypass_msg_recv_callback(bool is_ok, const char *noti_str)
{
    cPrintLog(CDBG_MAIN_LOG, "wifi noti %d, %s\n", is_ok, noti_str);
}

void cWifi_bypass_recv_data_callback(const uint8_t *data, uint32_t data_size)
{
    cPrintLog(CDBG_MAIN_LOG, "%s", data);
}

void cfg_examples_wifi_bypass(void)
{
    int wifi_result;
    int timeout;
    const char at_cmd[]="AT\r\n";

    //disable wifi info log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));

    cWifi_resource_init();  // Initalize resource for WIFI module 
    cWifi_prepare_start();  // prepare for WIFI module

    wifi_result = cWifi_bypass_req(cWifi_bypass_recv_data_callback, cWifi_bypass_msg_recv_callback);
    if(wifi_result == CWIFI_Result_OK)
    {
        timeout = 5000;
        while(!cWifiState_is_bypass_mode())
        {
            if(--timeout==0)break;  //wait bypassmode
            nrf_delay_ms(1);
        }
        cPrintLog(CDBG_MAIN_LOG, "WIFI bypass started! %d\n", timeout);

        if(timeout > 0)
        {
            cPrintLog(CDBG_MAIN_LOG, "send at cmd:%s", at_cmd);
            cWifiState_bypass_write_request(at_cmd, strlen(at_cmd));
            nrf_delay_ms(1000);
        }
        cWifi_abort_req();  //power off wifi module

        timeout = 5000;
        while(--timeout)
        {
            if(!cWifi_is_bypass_state() && !cWifi_bus_busy_check())break;  //wait bypassmode
            nrf_delay_ms(1);
        }
        cPrintLog(CDBG_MAIN_LOG, "WIFI bypass stopped! %d\n", timeout);
    }
    else
    {
        // WIFI not available or busy
        cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
    }
}
#endif
#endif

void cfg_examples_check_enter(void)
{
#if (CFG_EXAMPLES_TYPE_DEF != CFG_EXAMPLES_TYPE_NONE)
    uint32_t err_code;

    cPrintLog(CDBG_MAIN_LOG, "=====EXAMPLES_Start : %d=====\n", CFG_EXAMPLES_TYPE_DEF);
    main_examples_prepare();

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_SCAN_TWO_BSSID)
    cfg_examples_wifi_scan_two_bssid();
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_SCAN_FILTERED)
    cfg_examples_wifi_scan_filtered();
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_MAGNET_SENSOR)
    cfg_examples_magnet_sensor();
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_WKUP_GPIO)
    cfg_examples_wkup_gpio();
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_SIFOX)
    cfg_examples_sigfox();
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_GPS)
    cfg_examples_gps();
#endif

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_WIFI_BYPASS)
     cfg_examples_wifi_bypass();
#endif

    while(1)
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
    
#endif
}
