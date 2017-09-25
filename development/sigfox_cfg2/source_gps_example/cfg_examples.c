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
#define CFG_EXAMPLES_WIFI_TCP_VIA_BYPASS    8

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
    cfg_magnetic_sensor_init(NULL, NULL);
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

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_WIFI_TCP_VIA_BYPASS)
static uint8_t resp_cache[1024];
static unsigned int resp_cache_idx;
static uint8_t cmd_buf[128];
static unsigned int cmd_buf_idx;
static uint8_t bypass_remap_buffer[1024];

static const char ap_info[] = "\"wisoltst\",\"1357913579\"";
static const char tcp_connect_info[] = "\"TCP\",\"211.202.2.19\",80";
static const char tcp_send_data[] = "GET /esp_test.html HTTP/1.0\r\nHost: www.hardcopyworld.com\r\n\r\n";

static void cWifi_bypass_TCP_parse_net_cmds(const uint8_t *data, uint32_t data_size)
{
    if((resp_cache_idx+data_size+1) < sizeof(resp_cache))
    {
        memcpy(&resp_cache[resp_cache_idx], data, data_size);
        resp_cache_idx += data_size;
        resp_cache[resp_cache_idx] = 0;
    }
}

static void cWifi_bypass_TCP_resp_cache_clear(void)
{
    memset(resp_cache, 0, sizeof(resp_cache));
    resp_cache_idx = 0;
}

static bool cWifi_bypass_TCP_wait_resp(int timeout_sec, const char *wait_char, int more_wait_byte, int *get_wait_resp_end_idx)
{
    int timeout_tick = timeout_sec * 100;
    char *st_ptr;
    int st_idx;
    int wait_char_len = strlen(wait_char);

    while(--timeout_tick > 0)
    {
        nrf_delay_ms(10);
        if(wait_char_len < resp_cache_idx)
        {
            st_ptr = strstr((char *)resp_cache, wait_char);
            if(st_ptr)
            {
                st_idx = st_ptr - (char *)resp_cache;
                if(st_idx + wait_char_len + more_wait_byte <= resp_cache_idx)
                {
                    if(get_wait_resp_end_idx)*get_wait_resp_end_idx = st_idx+wait_char_len;
                    return true;
                }
            }
        }
    }

    return false;
}

void cfg_examples_wifi_TCP_via_bypass(void)
{
    int wifi_result;
    int timeout;
    int recv_data_idx;

    //disable wifi info log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));

    cWifi_resource_init();  // Initalize resource for WIFI module 
    cWifi_prepare_start();  // prepare for WIFI module

    wifi_result = cWifi_bypass_req(cWifi_bypass_TCP_parse_net_cmds, NULL);
    if(wifi_result == CWIFI_Result_OK)
    {
        timeout = 5000;
        while(!cWifiState_is_bypass_mode())
        {
            if(--timeout==0)break;  //wait bypassmode
            nrf_delay_ms(1);
        }
        cWifiState_bypass_buffer_remapping(bypass_remap_buffer, sizeof(bypass_remap_buffer));
        cPrintLog(CDBG_MAIN_LOG, "WIFI TCP via bypass started!\n");
        if(timeout == 0)goto end_function;

        //make at command
        cmd_buf_idx = sprintf((char *)cmd_buf, "ATE0");
        cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
        //wait bypass ready
        while(!cWifiState_is_bypass_ready());
        cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
        //wait response
        if(!cWifi_bypass_TCP_wait_resp(1, "OK", 0, NULL))goto error;
        cWifi_bypass_TCP_resp_cache_clear();
        cPrintLog(CDBG_MAIN_LOG, "ATE0 OK\n");

        //make at command
        cPrintLog(CDBG_MAIN_LOG, "Try join to %s\n", ap_info);
        cmd_buf_idx = sprintf((char *)cmd_buf, "AT+CWJAP=");
        memcpy(&cmd_buf[cmd_buf_idx], ap_info, sizeof(ap_info)-1);
        cmd_buf_idx += (sizeof(ap_info)-1);
        cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
        //wait bypass ready
        while(!cWifiState_is_bypass_ready());
        cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
        //wait response
        if(!cWifi_bypass_TCP_wait_resp(15, "WIFI GOT IP", 6 /*\r\nOK\r\n*/, NULL))goto error;       
        cWifi_bypass_TCP_resp_cache_clear();
        cPrintLog(CDBG_MAIN_LOG, "AP join OK\n");

        //make at command
        cPrintLog(CDBG_MAIN_LOG, "Try connect to %s\n", tcp_connect_info);
        cmd_buf_idx = sprintf((char *)cmd_buf, "AT+CIPSTART=");
        memcpy(&cmd_buf[cmd_buf_idx], tcp_connect_info, sizeof(tcp_connect_info)-1);
        cmd_buf_idx += (sizeof(tcp_connect_info)-1);
        cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
        //wait bypass ready
        while(!cWifiState_is_bypass_ready());
        cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
        //wait response
        if(!cWifi_bypass_TCP_wait_resp(15, "CONNECT", 6 /*\r\nOK\r\n*/, NULL))goto error;       
        cWifi_bypass_TCP_resp_cache_clear();
        cPrintLog(CDBG_MAIN_LOG, "TCP connect OK\n");
       
        //make at command
        cPrintLog(CDBG_MAIN_LOG, "Try http GET\n", tcp_connect_info);
        cPrintLog(CDBG_MAIN_LOG, "tcp send data size:%d\n", (sizeof(tcp_send_data)-1));
        cmd_buf_idx = sprintf((char *)cmd_buf, "AT+CIPSEND=%d", (sizeof(tcp_send_data)-1));
        cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
        //wait bypass ready
        while(!cWifiState_is_bypass_ready());
        cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
        //wait response
        if(!cWifi_bypass_TCP_wait_resp(1, ">", 0, NULL))goto error;
        cWifi_bypass_TCP_resp_cache_clear();
        cPrintLog(CDBG_MAIN_LOG, "CIPSEND OK\n");
        memcpy(cmd_buf, tcp_send_data, (sizeof(tcp_send_data)-1));
        cmd_buf_idx = (sizeof(tcp_send_data)-1);
        cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
        //wait bypass ready
        while(!cWifiState_is_bypass_ready());
        cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
        //wait response
        if(!cWifi_bypass_TCP_wait_resp(10, "SEND OK\r\n", 0, &recv_data_idx))goto error;   
        cPrintLog(CDBG_MAIN_LOG, "SEND TCP OK %d, %d\n", resp_cache_idx, recv_data_idx);
        
        nrf_delay_ms(2000);  //wait for http response
        if(resp_cache_idx > recv_data_idx)
        {
            cPrintLog(CDBG_MAIN_LOG, "Data received. size:%d\n", (resp_cache_idx-recv_data_idx));
            cDataDumpPrintOut(CDBG_MAIN_LOG, &resp_cache[recv_data_idx], (resp_cache_idx-recv_data_idx));  //http://www.rapidtables.com/convert/number/hex-to-ascii.htm
        }

        goto end_function;
    }
    else
    {
        // WIFI not available or busy
        cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
        return;
    }


error:
    cPrintLog(CDBG_MAIN_LOG, "WIFI TCP via bypass error\n");

end_function:
    cWifi_abort_req();  //power off wifi module
    timeout = 5000;
    while(--timeout)
    {
        if(!cWifi_is_bypass_state() && !cWifi_bus_busy_check())break;  //wait bypassmode
        nrf_delay_ms(1);
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

#if (CFG_EXAMPLES_TYPE_DEF == CFG_EXAMPLES_WIFI_TCP_VIA_BYPASS)
     cfg_examples_wifi_TCP_via_bypass();
#endif

    while(1)
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
    
#endif
}
