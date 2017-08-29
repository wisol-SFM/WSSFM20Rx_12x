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
 * @brief control wifi module.
 *
 * This file contains the control wifi module.
 */


#ifndef __CFG_WIFI_MODULE_H__
#define __CFG_WIFI_MODULE_H__
#include "cfg_board_def.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CWIFI_Result_OK            (0)
#define CWIFI_Result_Busy          (-1)
#define CWIFI_Result_NoDevice      (-2)
#define CWIFI_Result_NotReady      (-3)
#define CWIFI_Result_NoData        (-4)
#define CWIFI_Result_NotStarted    (-5)
#define CWIFI_Result_NotSupported  (-6)

#define CWIFI_SPI_BUF_SIZE 80
#define CWIFI_AT_CMD_SIZE_MAX 256
#define CWIFI_AT_CMD_RESP_PACKET_MAX 63  // & 0x3F : 6bit max value

#define CWIFI_BSSID_CNT 2
#define CWIFI_BSSID_SIZE 6
#define CWIFI_SSID_SIZE 32
#define CWIFI_SSID_HEADER_SIZE_MAX 16

#define CWIFI_PROC_BOOT_TIME_MS 500

#define CWIFI_PROC_REQ_TIME_MS 10
#define CWIFI_MODULE_DETECT_TIME_MS 100
#define CWIFI_MODULE_DETECT_TIME_OUT_TICK 5
#define CWIFI_MODULE_DETECT_TRY_MAX 3

#define CWIFI_MODULE_ENTER_SUSPEND_MS 0 //0 sec 5000 // 5 Sec

#define CWIFI_SPI_TX_DATA_SEND_SIZE 32

#define CWIFI_SPI_WAIT_TIME_MS 10
#define CWIFI_SPI_WAIT_TIME_OUT_TICK 20  // 200 ms (CWIFI_SPI_WAIT_TIME_MS * CWIFI_SPI_WAIT_TIME_OUT_TICK)
#define CWIFI_SPI_ISR_CLEAR_WAIT_TIMEOUT_TICK 1
#define CWIFI_SPI_INTERRUPT_CLEAR_WAIT_TIME_MS 20

#define CWIFI_SPI_CMD_RESP_TIME_OUT_TICK_LONG 500  //for first response time out tick
#define CWIFI_SPI_CMD_RESP_WAIT_TIME_MS_LONG 20  //for first response time out time
#define CWIFI_SPI_CMD_RESP_TIME_OUT_TICK 1
#define CWIFI_SPI_CMD_RESP_WAIT_TIME_MS 10

#define CWIFI_SCAN_RETRY_WAIT_TIME_MS 2000
#define CWIFI_BYPASS_WAIT_TIME_MS 100

typedef void (*cWifi_bypass_recv_handler_t)(const uint8_t *pData, uint32_t dataSize);
typedef void (*cWifi_bypass_noti_callback_t)(bool is_ok, const char *noti_str);

/**
 * @brief       Function for initializing resource of wifi module 
 */
void cWifi_resource_init(void);
/**
 * @brief       Function for prepare wifi module
 */
void cWifi_prepare_start(void);

/**
 * @brief       Function for wifi scan time.
 * @param[in]   sec of the scan time
 *
 */
void cWifi_set_retry_time(unsigned int retry_time_sec);  //ref module_parameter_t.wifi_scan_retry_time_sec and  module_parameter_item_wifi_scan_retry_time_sec

/**
 * @brief       Function for entering wifi download mode
 */
void cWifi_enter_download_mode(void);

/**
 * @brief       Function for entering wifi rftest mode (Note: You can not perform other functions.)
 */
void cWifi_enter_rftest_mode(void);

/**
 * @brief       Function for power control wifi module
 */
void cWifi_power_control(bool on);

/**
 * @brief       Function for request scan work.
 *
 * @return      CWIFI_Result_OK on success. Otherwise an error code(eg.CWIFI_Result_Busy).
 */
int cWifi_ap_scan_req(void);

/**
 * @brief       Function for request scan work.
 *
 * @param[in]   p_SSID_header_str pointer of SSID Head String (for SSID Filter)
 * @return      CWIFI_Result_OK on success. Otherwise an error code(eg.CWIFI_Result_Busy).
 */
int cWifi_ap_get_available_first_BSSID(const char *p_SSID_header_str);

/**
 * @brief       Function for bypass mode (for i2c slavle control).
 *
 * @return      CWIFI_Result_OK on success. Otherwise an error code(eg.CWIFI_Result_Busy).
 * @param[callback function]   recv_CB wifi recv data callback
 * @param[callback function]   noti_CB state notification
 */
int cWifi_bypass_req(cWifi_bypass_recv_handler_t recv_CB, cWifi_bypass_noti_callback_t noti_CB);


/**
 * @brief       Function for aborting wifi module
 */
void cWifi_abort_req(void);

/**
 * @brief       Function for status check (bus use check)
 */
bool cWifi_bus_busy_check(void);

/**
 * @brief       Function for status check (scaning state check)
 */
bool cWifi_is_scan_state(void);

/**
 * @brief       Function for status check (woring bypass mode)
 */
bool cWifi_is_bypass_state(void);

/**
 * @brief       Function for status check (available check)
 */
bool cWifi_is_detected(void);

/**
 * @brief       Function for busy check
 */
bool cWifi_is_busy(void);

/**
 * @brief       Function for version check
 */
bool cWifi_get_version_info(uint8_t *appVer, uint16_t *initDataVer);


/**
 * @brief       Function for getting scan result.
 *
 * @param[out]   bssid pointer of getting bssid
 *
 * @return      CWIFI_Result_OK on success. Otherwise an error code(eg.CWIFI_Result_NoData).
 */
int cWifi_get_BSSIDs_bufPtr(uint8_t **bssid);  //data buf size is 6byte*getCnt CWIFI_BSSID_CNT*CWIFI_BSSID_SIZE

/**
 * @brief       Function for getting scan result.
 *
 * @param[out]   get_cnt pointer of getting scan count
 * @param[out]   ssid pointer of getting ssid
 * @param[out]   rssi pointer of getting rssi
 * @param[out]   bssid pointer of getting bssid
 * @return      CWIFI_Result_OK on success. Otherwise an error code
 */
int cWifi_get_scan_result(uint32_t *get_cnt, uint8_t **ssid /*CWIFI_SSID_SIZE*/, int32_t **rssi, uint8_t **bssid/*CWIFI_BSSID_SIZE*/);

/**
 * @brief       Function for getting scan result (empty check).
 *
 * @return      true : scan result is empty.
 */
bool cWifi_BSSID_null_check(void);

#ifdef FEATURE_CFG_BYPASS_CONTROL
/**
 * @brief       Function for bypass data to wifi module
 * @param[in]   pData pointer of send data
 * @param[in]   dataSize size of send data
 *
 * @return      boolean result of data send request to wifi module
 */
bool cWifiState_bypass_write_request(const char *pData, unsigned int dataSize);

/**
 * @brief       Function for status check (available check)
 */
bool cWifiState_is_bypass_mode(void);
#endif

/**
 * @brief       wifi driver Function for wifi init
 * @return      CWIFI_Result_OK on success 
 */
int wifi_drv_init(void);
/**
 * @brief       wifi driver Function for wifi scan start
 * @return      CWIFI_Result_OK on success 
 */
int start_AP_scan(void);
/**
 * @brief       wifi driver Function for wifi scan start
 * @param[in]   interval (sec 1~60)
 */
void set_scan_interval(int interval);
/**
 * @brief       wifi driver Function for result of wifi scan
 * @param[out]   get_cnt pointer of getting scan count
 * @param[out]   ssid pointer of getting ssid
 * @param[out]   rssi pointer of getting rssi
 * @param[out]   bssid pointer of getting bssid
 * @return      CWIFI_Result_OK on success 
 */
int get_AP_scanResult(uint32_t *get_cnt, uint8_t **ssid /*CWIFI_SSID_SIZE*/, int32_t **rssi, uint8_t **bssid/*CWIFI_BSSID_SIZE*/);
//set_power_mode(int mode) //not support
//Able to join network using credentials supplied by user application.  //not support
//Able to upload and download data from network.  //not support
//getter and setter functions compatible with WIFI standard.  //not support

#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
