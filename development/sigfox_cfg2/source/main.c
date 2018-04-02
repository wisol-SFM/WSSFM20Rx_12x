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
 * @brief tracking Sample Application main.c file.
 *
 * This file contains the source code for an tracking sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "cfg_board_def.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_flash.h"
#include "ble_bas.h"

#include "cfg_scenario.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "peer_manager.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "cfg_app_main.h"
#include "cfg_sigfox_module.h"
#include "cfg_bma250_module.h"
#include "cfg_tmp102_module.h"
#include "cfg_gps_module.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"
#include "cfg_board.h"
#include "nrf_drv_gpiote.h"
#include "fstorage.h"
#include "fds.h"
#include "nrf_drv_twi.h"
#include "nfc_t2t_lib.h"
#include "nfc_uri_msg.h"
#include "nfc_launchapp_msg.h"
#include "nfc_launchapp_rec.h"
#include "nfc_ndef_msg.h"
#include "nfc_text_rec.h"
#include "nrf_drv_saadc.h"
#include "hardfault.h"
#include "ble_dfu.h"
#include "ble_nus.h"
#include "nrf_drv_clock.h"
#include "cfg_external_sense_gpio.h"

//#define TEST_FEATURE_SIGFOX_CW
#ifdef TEST_FEATURE_SIGFOX_CW
#define SIGFOX_CW_CMD "AT$CW=868130000,1,15\r\n"
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (CDEV_MODULE_TYPE == CDEV_MODULE_SFM40R)
#define DEVICE_NAME                     "WISOL_SFM40R"                           /**< Name of device. Will be included in the advertising data. */
#elif (CDEV_MODULE_TYPE == CDEV_MODULE_SFM50R)
#define DEVICE_NAME                     "WISOL_SFM50R"                           /**< Name of device. Will be included in the advertising data. */
#elif (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)
#define DEVICE_NAME                     "WISOL_SFM60R"                           /**< Name of device. Will be included in the advertising data. */
#else
#define DEVICE_NAME                     "WISOL_SFM21R"                           /**< Name of device. Will be included in the advertising data. */
#endif
#define MANUFACTURER_NAME               "WISOL_Corp"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define ASSETTRACKER_NAME               "ihere"
#define MINI_ASSETTRACKER_NAME          "ihere mini"
#define M3_ASSETTRACKER_NAME            "M3"
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      30                                         /**< The advertising timeout in units of seconds. */

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define TX_POWER_LEVEL                    (4)

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                            /**< Handle of the current connection. */

static ble_dfu_t m_dfus;                                                            /**< Structure used to identify the DFU service. */

#define ONE_DAY_SEC (60*60*24)
#ifdef CDEV_NUS_MODULE
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
#else
//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */
#endif

#if (CDEV_MODULE_TYPE == CDEV_MODULE_SFM50R)
extern int16_t tmp102a, tmp102b;
#endif
extern uint8_t tmp102a_sigfox, tmp102b_sigfox;

static bool m_softdevice_init_flag;
static bool m_hitrun_test_flag = false;

bool ble_connect_on = false;
extern void cfg_examples_check_enter(void);

/**@brief Callback function for events in the DFU.
 *
 * @details This function will be called in case of an assert in the SoftDevice.

 * @param[in] p_dfu            Forward declaration of the ble_nus_t type.
 * @param[in] p_evt            
 */

static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_DFU_EVT_INDICATION_DISABLED:
            cPrintLog(CDBG_BLE_INFO, "Indication for BLE_DFU is disabled\r\n");
            break;

        case BLE_DFU_EVT_INDICATION_ENABLED:
            cPrintLog(CDBG_BLE_INFO, "Indication for BLE_DFU is enabled\r\n");
            break;

        case BLE_DFU_EVT_ENTERING_BOOTLOADER:
            cPrintLog(CDBG_BLE_INFO, "Device is entering bootloader mode!\r\n");
            break;
        default:
            cPrintLog(CDBG_BLE_INFO, "Unknown event from ble_dfu\r\n");
            break;
    }
}


#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */
#define USE_UICR_FOR_MAJ_MIN_VALUES 1
#define BATTERY_LEVEL_AVG_CNT 3

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

unsigned int main_schedule_tick = 0;

APP_TIMER_DEF(m_main_timer_id);

#ifdef CDEV_NUS_MODULE
//APP_TIMER_DEF(m_nus_timer_id);
#endif

extern uint8_t frame_data[(SIGFOX_SEND_PAYLOAD_SIZE*2)+1];  //for hexadecimal

module_mode_t m_module_mode;
module_mode_t m_module_mode_old;
bool m_init_excute;
module_parameter_t m_module_parameter;
bool m_module_parameter_update_req;
bool m_module_parameter_save_N_reset_req;
bool m_module_parameter_fs_init_flag = false;

int m_module_ready_wait_timeout_tick = 0;

volatile bool main_wakeup_interrupt;
volatile int main_wakeup_reason = main_wakeup_reason_normal;  //main_wakeup_reason_type
#ifdef PIN_DEF_BUTTON  //CDEV_BOARD_M3
volatile bool main_button_detected;
#endif
volatile bool main_wkup_push_detected;
volatile bool main_magnet_detected;
volatile bool main_ACC_ISR_detected;

volatile bool mnfc_tag_on = false;
volatile bool mnfc_init_flag = false;
main_nfg_tag_on_callback mnfc_tag_on_CB = NULL;
volatile bool main_powerdown_request = false;
#ifdef PIN_DEF_GPS_BKUP_CTRL_WITH_PULLUP  //GPS_BKUP_CTRL
int main_GPS_BKUP_time_out = 0;  //GPS_BKUP_CTRL
#endif
unsigned int main_Sec_tick = 0;

#ifdef CDEV_NUS_MODULE
nus_service_parameter_t m_nus_service_parameter;
#endif
module_peripheral_data_t m_module_peripheral_data;
module_peripheral_ID_t m_module_peripheral_ID;

volatile bool nus_parameter_update = false;

#define MAX_REC_COUNT      3     /**< Maximum records count. */

/** @snippet [NFC Launch App usage_0] */
/* nRF Toolbox Android application package name */
#if 0
static const uint8_t m_android_package_name[] = {'n', 'o', '.', 'n', 'o', 'r', 'd', 'i', 'c', 's',
                                                 'e', 'm', 'i', '.', 'a', 'n', 'd', 'r', 'o', 'i',
                                                 'd', '.', 'n', 'r', 'f', 't', 'o', 'o', 'l', 'b',
                                                 'o', 'x'};
#else
#if 0
static const uint8_t m_android_package_name[] = {'h','t','t','p','s',':','/','/','p','l','a','y','.',
                                                'g','o','o','g','l','e','.','c','o','m','/','s','t',
                                                'o','r','e','/','a','p','p','s','/','d','e','t','a',
                                                'i','l','s','?','i','d','=','n','o','.','n','o','r',
                                                'd','i','c','s','e','m','i','.','a','n','d','r','o',
                                                'i','d','.','n','r','f','t','o','o','l','b','o','x',
                                                '&','h','l','=','e','n'};
#endif
#endif
/* nRF Toolbox application ID for Windows phone */
uint8_t m_ndef_msg_buf[256];

#ifdef PIN_DEF_BATTERY_ADC_INPUT
APP_TIMER_DEF(m_battery_timer_id);                                               /**< Battery measurement timer. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     600                                          /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      6                                            /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS 1200                                         /**< Value in millivolts for voltage used as reference in ADC conversion on NRF51. */
#define ADC_INPUT_PRESCALER               3                                            /**< Input prescaler for ADC convestion on NRF51. */
#define ADC_RES_10BIT                     1024                                         /**< Maximum digital value for 10-bit ADC conversion. */

#if (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 5/3)
#elif  (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) 
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 5/3)
#elif  (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2) 
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 168/100)
#elif  (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI)
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 5/3)
#elif  (CDEV_BOARD_TYPE == CDEV_BOARD_M3) 
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 168/100)
#else
#define ADJUST_BATTERY_VALUE    0 //1344 //270                                          /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 5/3)
#endif

//static ble_bas_t                        m_bas;                                   /**< Structure used to identify the battery service. */
static nrf_saadc_value_t adc_buf[2];
uint8_t   avg_report_volts;
uint16_t total_batt_lvl_in_milli_volts;
#else
uint8_t   avg_report_volts;
uint16_t total_batt_lvl_in_milli_volts;
#endif

static bool nus_service = false;

void nfc_uninit(void);
void main_set_module_state(module_mode_t state);
void advertising_start(bool start_flag, bool led_ctrl_flag);
bool main_schedule_state_is_idle(void);
void main_timer_schedule_stop(void);
void module_parameter_check_update(void);
void nus_send_data(char module);
uint32_t nus_send_id_pac_mac(uint8_t * send_buffer);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#ifdef PIN_DEF_BATTERY_ADC_INPUT
/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    #ifdef ADC_PRESENT
    nrf_drv_adc_sample();
    #else // SAADC_PRESENT
    uint32_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
    #endif // ADC_PRESENT
}

void cfg_bas_timer_create(void)
{
    uint32_t   err_code;

    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void cfg_bas_timer_start(void)
{
    uint32_t    err_code;

    avg_report_volts = 0;
    total_batt_lvl_in_milli_volts = 0;
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void un_adc_configure(void)
{
//    nrf_drv_saadc_abort();
    nrf_drv_saadc_channel_uninit(0);
    nrf_drv_saadc_uninit();
}

#if 0
/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = on_bas_evt;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}
#endif

uint16_t get_avg_batt_lvl_in_milli_volts(void)
{
    return (total_batt_lvl_in_milli_volts/BATTERY_LEVEL_AVG_CNT);
}

 /**@brief Function for handling the ADC interrupt.
  *
  * @details  This function will fetch the conversion result from the ADC, convert the value into
  *           percentage and send it to peer.
  */
 void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
 {
    static uint8_t  check_count = 0;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t   adc_result;
        uint16_t            batt_lvl_in_milli_volts;
        uint32_t            err_code;

        adc_result = p_event->data.done.p_buffer[0];

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) + ADJUST_BATTERY_VALUE;
        if(check_count++ < BATTERY_LEVEL_AVG_CNT)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
            APP_ERROR_CHECK(err_code);

            total_batt_lvl_in_milli_volts += batt_lvl_in_milli_volts;
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            avg_report_volts = ((get_avg_batt_lvl_in_milli_volts()) / 100);
            check_count = 0;
            un_adc_configure();
            m_nus_service_parameter.battery_volt[0] = avg_report_volts;
            m_nus_service_parameter.module='B';
//            nus_send_data('B');
        }
        cPrintLog(CDBG_EXT_SEN_INFO, "BATTERY %d %d %d \n",batt_lvl_in_milli_volts,avg_report_volts, check_count);
    }
 }

/**@brief Function for configuring ADC to do battery level conversion.
 */
void adc_configure(void)
{
    #ifdef ADC_PRESENT
    ret_code_t err_code = nrf_drv_adc_init(NULL, adc_event_handler);
    APP_ERROR_CHECK(err_code);

    static nrf_drv_adc_channel_t channel =
        NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_DISABLED);
    // channel.config.config.input = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    channel.config.config.input = (uint32_t)NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_drv_adc_channel_enable(&channel);

    err_code = nrf_drv_adc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);
    #else //  SAADC_PRESENT
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(PIN_DEF_BATTERY_ADC_INPUT);

    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
    #endif //ADC_PRESENT
}

#endif
#ifdef CDEV_NUS_MODULE
void nus_module_parameter_get(void)
{
    m_nus_service_parameter.report_count[0]= (ONE_DAY_SEC / m_module_parameter.idle_time)&0x000000FF;
    m_nus_service_parameter.wifi_scan_time[0] = (uint8_t)m_module_parameter.wifi_scan_retry_time_sec&0x000000FF;
    m_nus_service_parameter.gps_tracking_time[0] = (uint8_t)m_module_parameter.gps_acquire_tracking_time_sec&0x000000FF;
}

static void nus_data_init()
{
    memset(m_nus_service_parameter.gps_data,0,8);
    memset(m_nus_service_parameter.gps_cn0,0,1);
    memset(m_nus_service_parameter.wifi_data,0,12);
    memset(m_nus_service_parameter.temperature_data,0,2);
    m_nus_service_parameter.magnet_event = '0';
    m_nus_service_parameter.accellometer_event = '1';
}

/**@brief Callback function for time-out in NUS.
 *
 * @details This function will be called when the timer expires.

 * @param[in] p_context   unused parameter                  
 */

void nus_send_data(char module)
{
    static uint8_t nus_send_buffer[20];
    int strLen;
    uint32_t      err_code=0;
    memset(nus_send_buffer,0xFF,20);
    if(ble_connect_on) {
        switch(module)
        {
            case 'T':
                nus_send_buffer[0]='T';
                memcpy(&nus_send_buffer[1],m_nus_service_parameter.temperature_data,2);
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, 3);
                break;
            case 'G':
                nus_send_buffer[0]='G';
                memcpy(&nus_send_buffer[1],m_nus_service_parameter.gps_data,8);
                nus_send_buffer[9]= m_nus_service_parameter.gps_cn0[0];
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, 10);
                break;
            case 'W':
                nus_send_buffer[0]='W';
                memcpy(&nus_send_buffer[1],m_nus_service_parameter.wifi_data,6);
                nus_send_buffer[7]= m_nus_service_parameter.wifi_rssi[0];
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, 8);
                break;
            case 'B':
                nus_send_buffer[0]='B';
                nus_send_buffer[1]= m_nus_service_parameter.battery_volt[0];
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, 2);
                break;

            case 'A':
                nus_send_buffer[0]='A';
                memcpy(&nus_send_buffer[1],&m_nus_service_parameter.acc_x,2);
                memcpy(&nus_send_buffer[3],&m_nus_service_parameter.acc_y,2);
                memcpy(&nus_send_buffer[5],&m_nus_service_parameter.acc_z,2);
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, 7);
                break;
            case 'I':
                err_code = nus_send_id_pac_mac(nus_send_buffer);
                break;
            case 'V':  //send version
                nus_send_buffer[0]='V';
                strLen = sprintf((char *)&nus_send_buffer[1], "%s_V%s_%02x", (const char *)m_cfg_model_name, (const char *)m_cfg_sw_ver, m_cfg_board_type);
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, (strLen+1));
 //               strLen = sprintf((char *)&nus_send_buffer[1], "D %s", (const char *)m_cfg_build_date);
 //               err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, (strLen+1));
 //               strLen = sprintf((char *)&nus_send_buffer[1], "T %s", (const char *)m_cfg_build_time);
 //               err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, (strLen+1));
                break;
            case 'D':  //send build date
                nus_send_buffer[0]='D';
                strLen = sprintf((char *)&nus_send_buffer[1], "D %s", (const char *)m_cfg_build_date);
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, (strLen+1));
                strLen = sprintf((char *)&nus_send_buffer[1], "T %s", (const char *)m_cfg_build_time);
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, (strLen+1));
                break;
            case 'O':
                nus_module_parameter_get();
                nus_send_buffer[0]='O';
                nus_send_buffer[1]=m_nus_service_parameter.report_count[0];
                nus_send_buffer[2]=m_nus_service_parameter.wifi_scan_time[0];
                nus_send_buffer[3]=m_nus_service_parameter.gps_tracking_time[0];
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, 4);
                break;
            case 'Z':
                nus_send_buffer[0]='Z';
                err_code = ble_nus_string_send(&m_nus, (uint8_t*)&nus_send_buffer, 1);
                break;
            default:
                break;
        }

        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

#if 0
static void nus_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t      err_code;
//    cPrintLog(CDBG_MAIN_LOG, "NUS SEND DATA\n",  __func__, __LINE__);
    if(ble_connect_on)
    {
        err_code = ble_nus_string_send(&m_nus, (uint8_t*)m_nus_service_parameter.gps_data, 16);
        //    err_code = ble_nus_string_send(&m_nus, (uint8_t*)&m_nus_service_parameter, sizeof(m_nus_service_parameter));
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief timer create function for NUS.
 *
 * @details This function will be called when NUS starts.

 * @param[in]                   
 */

static void nus_timer_create()
{
    uint32_t      err_code;
    err_code = app_timer_create(&m_nus_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                nus_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief timer start function for NUS.
 *
 * @details This function will be called when NUS starts.

 * @param[in]                   
 */

static void nus_timer_start()
{
    uint32_t      err_code;

    err_code = app_timer_start(m_nus_timer_id, APP_TIMER_TICKS(1000*NUS_INTERVAL_TIME_DEFAULT, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief timer stop function for NUS.
 *
 * @details This function will be called when NUS stops.

 * @param[in]                   
 */

static void nus_timer_stop()
{
    uint32_t err_code;

    err_code = app_timer_stop(m_nus_timer_id);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief callback function for ble event in NUS.
 *
 * @details This function will be called when ble is connected

 * @param[in]                   
 */

static void on_nus_evt(ble_nus_t * p_nus, ble_evt_t * p_ble_evt)
{
    if ((p_nus == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
 //           nus_timer_start();
            break;

        case BLE_GAP_EVT_DISCONNECTED:      
//            nus_timer_stop();
            if(nus_service)
            {
                ihere_mini_fast_schedule_start();
                nus_service = false;
            }
            if(nus_parameter_update)
            {
                nus_parameter_update = false;
                m_module_parameter_update_req = true;
                module_parameter_check_update();

                nrf_delay_ms(1000);
                NVIC_SystemReset();
            }
            break;
        case BLE_GATTS_EVT_WRITE:
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            break;
        default:
            // No implementation needed.
            break;
    }

}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */

extern bool mnus_acc_report;
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
//    volatile uint32_t err_code;
//    uint8_t respose[4]="OK";

    if(length > 0)
    {
        switch(p_data[0])
        {
            case 'T':
                nus_send_data('I');
                main_wakeup_interrupt = true;
                main_wakeup_reason = main_wakeup_reason_ble_event;
//                err_code = ble_nus_string_send(&m_nus, (uint8_t*)respose, 2);
                break;

            case 'L':
                nus_service = true;
                ihere_mini_normal_schedule_mode_change();
//                nus_timer_start();
                break;

            case 'E':
                nus_service = false;
                break;

            case 'S':
                mnus_acc_report = false;
                bma250_set_state(IO_SETUP);
                cfg_bma250_timers_stop();
                cfg_bma250_timers_start();
                break;

            case 'A' :
#ifdef FEATURE_CFG_ACC_REPORT
                mnus_acc_report = true;
                bma250_set_state(IO_SETUP);
                cfg_bma250_timers_stop();
                cfg_bma250_timers_start();
#endif
                break;

            case 'V':  //read version
                nus_send_data('V');
//                break;
//              case 'O': 
                nus_send_data('O');
                break;

            case 'D':  //build date
                nus_send_data('D');
                break;

            case 'Z':
                m_module_parameter.idle_time=(ONE_DAY_SEC/p_data[1]);
                m_module_parameter.wifi_scan_retry_time_sec = p_data[2];
                m_module_parameter.gps_acquire_tracking_time_sec = p_data[3];
                m_module_parameter_save_N_reset_req = true;
                break;

            default:
                break;
        }
    }
}
/**@snippet [Handling the data received over BLE] */

#endif

/**@brief Function for handling the data from FS.
 *
 * @details This function will set the data received from FS
 *          
 *
 * @param[in] item  configuration item
 */
unsigned int main_get_param_val(module_parameter_item_e item)
{
    unsigned int ret = 0;
    switch(item)
    {
        case module_parameter_item_idle_time:
            ret = m_module_parameter.idle_time;
            break;

        case module_parameter_item_beacon_interval:
            ret = m_module_parameter.beacon_interval;
            break;

        case module_parameter_item_wifi_scan_retry_time_sec:
            ret = m_module_parameter.wifi_scan_retry_time_sec;
            break;

        case module_parameter_item_start_wait_time_for_board_control_attach_sec:
            ret = m_module_parameter.start_wait_time_for_board_control_attach_sec;
            break;

        case module_parameter_item_start_wait_time_for_ctrl_mode_change_sec:
            ret = m_module_parameter.start_wait_time_for_ctrl_mode_change_sec;
            break;

        case module_parameter_item_gps_tracking_time_sec:
            ret = m_module_parameter.gps_acquire_tracking_time_sec;
            break;

        case module_parameter_item_boot_nfc_unlock:
            ret = m_module_parameter.boot_nfc_unlock;
            break;

        case module_parameter_item_fota_enable:
            ret = m_module_parameter.fota_enable;
            break;

        case module_parameter_item_scenario_mode:
            ret = m_module_parameter.scenario_mode;
            break;

        case module_parameter_item_magnetic_gpio_enable:
            ret = m_module_parameter.magnetic_gpio_enable;
            break;

        case module_parameter_item_wkup_gpio_enable:
            ret = m_module_parameter.wkup_gpio_enable;
            break;

        case module_parameter_item_wifi_testmode_enable:  //not used
            ret = m_module_parameter.wifi_testmode_enable;  //not used
            break;

        case module_parameter_item_snek_testmode_enable:
            ret = m_module_parameter.sigfox_snek_testmode_enable;
            break;

        case module_parameter_item_gps_cn0_current_savetime_enable:
            ret = m_module_parameter.cgps_cn0_current_savetime_enable;
            break;

        default:
            break;
    }
    return ret;
}

/**@brief Function for handling the data from FS.
 *
 * @details This function will get the data received from FS
 *          
 *
 * @param[in] item  configuration item
 */

void main_set_param_val(module_parameter_item_e item, unsigned int val)
{
    switch(item)
    {
        case module_parameter_item_idle_time:
            cPrintLog(CDBG_MAIN_LOG, "wakeup time:%d to %d\n", m_module_parameter.idle_time, val);
            m_module_parameter.idle_time = (uint32_t)val;
            break;

        case module_parameter_item_beacon_interval:
            cPrintLog(CDBG_MAIN_LOG, "BLE beacon interval:%d to %d\n", m_module_parameter.beacon_interval, val);
            m_module_parameter.beacon_interval = (uint32_t)val;
            break;

        case module_parameter_item_wifi_scan_retry_time_sec:
            m_module_parameter.wifi_scan_retry_time_sec = (uint32_t)val;
            cPrintLog(CDBG_MAIN_LOG, "wifi scan retry time:%d to %d\n", m_module_parameter.wifi_scan_retry_time_sec, val);
#ifdef CDEV_WIFI_MODULE
            cWifi_set_retry_time(m_module_parameter.wifi_scan_retry_time_sec);
#endif
            break;
            
        case module_parameter_item_start_wait_time_for_board_control_attach_sec:
            cPrintLog(CDBG_MAIN_LOG, "sfm_boot_mode:%d to %d\n", m_module_parameter.start_wait_time_for_board_control_attach_sec, val);  //use to sfm_boot_mode START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC
            cPrintLog(CDBG_MAIN_LOG, "mode def:0:normal, 1:wifi rf test, 2:wifi always on, 3:ble test, 4:gps test mode\n");
            cPrintLog(CDBG_MAIN_LOG, "mode def:5:wifi rf test bridge from RTT to uart, 6:sigfox over RTT, 7:sigfox over Uart, 8:WIFI AP(SFMTEST0000) and BLE BEACON(SFMTEST0000)\n");
            m_module_parameter.start_wait_time_for_board_control_attach_sec = (uint32_t)val;
            break;

        case module_parameter_item_start_wait_time_for_ctrl_mode_change_sec:
            cPrintLog(CDBG_MAIN_LOG, "sigfox DL:%d to %d\n", m_module_parameter.start_wait_time_for_ctrl_mode_change_sec, val);
            m_module_parameter.start_wait_time_for_ctrl_mode_change_sec = (uint32_t)val;
            break;

        case module_parameter_item_gps_tracking_time_sec:
            cPrintLog(CDBG_MAIN_LOG, "gps acquire tracking time:%d to %d\n", m_module_parameter.gps_acquire_tracking_time_sec, val);
            m_module_parameter.gps_acquire_tracking_time_sec = (uint32_t)val;
            break;

        case module_parameter_item_boot_nfc_unlock:
            cPrintLog(CDBG_MAIN_LOG, "booting enter power down:%d to %d\n", m_module_parameter.boot_nfc_unlock, val);
            m_module_parameter.boot_nfc_unlock = (bool)val;
            break;

        case module_parameter_item_fota_enable:
            cPrintLog(CDBG_MAIN_LOG, "ble fota(DFU):%d to %d\n", m_module_parameter.fota_enable, val);
            m_module_parameter.fota_enable = (bool)val;
            break;

        case module_parameter_item_scenario_mode:
            cPrintLog(CDBG_MAIN_LOG, "scenario type:%d to %d\n",  m_module_parameter.scenario_mode, val);
            m_module_parameter.scenario_mode = (uint16_t)val;
            break;

        case module_parameter_item_magnetic_gpio_enable:
            cPrintLog(CDBG_MAIN_LOG, "magnetic:%d to %d\n",  m_module_parameter.magnetic_gpio_enable, val);
            m_module_parameter.magnetic_gpio_enable = (bool)val;
            break;

        case module_parameter_item_wkup_gpio_enable:
            cPrintLog(CDBG_MAIN_LOG, "wakeup key:%d to %d\n",  m_module_parameter.wkup_gpio_enable, val);
            m_module_parameter.wkup_gpio_enable = (char)val;
            break;

        case module_parameter_item_wifi_testmode_enable:  //use to disable_battery_power_down
            cPrintLog(CDBG_MAIN_LOG, "use to disable battery power down:%d to %d\n",  m_module_parameter.wifi_testmode_enable, val);
            m_module_parameter.wifi_testmode_enable = (bool)val;  //use to disable_battery_power_down
            break;

        case module_parameter_item_snek_testmode_enable:
            cPrintLog(CDBG_MAIN_LOG, "snek test mode:%d to %d\n",  m_module_parameter.sigfox_snek_testmode_enable, val);
            m_module_parameter.sigfox_snek_testmode_enable = (bool)val;
            break;

        case module_parameter_item_gps_cn0_current_savetime_enable:
            cPrintLog(CDBG_MAIN_LOG, "cgps_cn0_current_savetime_enable:%d to %d\n",  m_module_parameter.cgps_cn0_current_savetime_enable, val);
            m_module_parameter.cgps_cn0_current_savetime_enable = (bool)val;
            break;

        default:
            break;
    }
}

/**@brief Function for handling the data from default value.
 *
 * @details This function will set the data if upgrading configuraion item
 *          
 */

static void module_parameter_default_init(void)
{
    memset(&m_module_parameter, 0, sizeof(m_module_parameter));
    m_module_parameter.magic_top = MODULE_PARAMETER_MAGIC_TOP;

    m_module_parameter.idle_time = MAIN_IDLE_TIME_DEFAULT;
    m_module_parameter.beacon_interval = BEACON_INTERVAL_TIME_DEFAULT;
    m_module_parameter.wifi_scan_retry_time_sec =  CWIFI_SEC_RETRY_TIMEOUT_SEC_DEFAULT;
    m_module_parameter.start_wait_time_for_board_control_attach_sec = START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC;
    m_module_parameter.start_wait_time_for_ctrl_mode_change_sec = START_WAIT_TIME_FOR_CTRL_MODE_CHANGE_SEC;  //use to sigfox dl enable
    m_module_parameter.gps_acquire_tracking_time_sec = CGPS_ACQUIRE_TRACKING_TIME_SEC;
    m_module_parameter.boot_nfc_unlock = MAIN_BOOT_NFC_UNLOCK_DEFAULT;
    m_module_parameter.fota_enable = MAIN_FOTA_ENABLE_DEFAULT;
    m_module_parameter.scenario_mode = MAIN_SCENARIO_MODE_DEFAULT;
    m_module_parameter.magnetic_gpio_enable = MAIN_MAGNETIC_GPIO_ENABLE_DEFAULT;  //MAIN_MAGNETIC_GPIO_ENABLE_DEFAULT
    m_module_parameter.wkup_gpio_enable = MAIN_WKUP_GPIO_ENABLE_DEFAULT;  //MAIN_WKUP_GPIO_ENABLE_DEFAULT
    m_module_parameter.wifi_testmode_enable = 0; //MAIN_WIFI_TESTMODE_ENABLE_DEFAULT;  //use to disable_battery_power_down
    m_module_parameter.sigfox_snek_testmode_enable = MAIN_SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT;
    m_module_parameter.cgps_cn0_current_savetime_enable = CGPS_CN0_CURRENT_SAVETIME_ENABLE;

    m_module_parameter.magic_bottom = MODULE_PARAMETER_MAGIC_BOTTOM;
    m_module_parameter.guard_area_align4 = 0;
    
    m_module_parameter.board_ID = m_cfg_board_type;
}

static void module_parameter_fs_evt_handler(fs_evt_t const * const evt, fs_ret_t result);
FS_REGISTER_CFG(fs_config_t param_fs_config) =
{
    .callback  = module_parameter_fs_evt_handler, // Function for event callbacks.
    .num_pages = 1,      // Number of physical flash pages required.
    .priority  = 0xFE            // Priority for flash usage.
};

/**@brief Function for handling event from FS.
 *
 * @details This function will get the result received from FS
 *          
 *
 * @param[in] evt  unused
 * @param[in] result  fstorage return values
 */

static void module_parameter_fs_evt_handler(fs_evt_t const * const evt, fs_ret_t result)
{
    module_parameter_t *p_setting_val;
    if (result == FS_SUCCESS)
    {
        cPrintLog(CDBG_FLASH_INFO, "FS Evt OK! %d\n", evt->id);
        p_setting_val = (module_parameter_t *)(param_fs_config.p_start_addr);
        if(p_setting_val->magic_top == 0)  //factory reset
        {
            cPrintLog(CDBG_FLASH_INFO, "target reset for factory reset\n");
            cfg_board_reset();
        }
    }
    else
    {
        // An error occurred.
        cPrintLog(CDBG_FLASH_ERR, "FS Evt Error! %d, %d\n", evt->id, result);
    }
}

/**@brief Function for handling address from FS.
 *
 * @details This function will get the address for writing configuration item
 *          
 *
 * @param[in] page_num  page number for writing configuration 
 * 
 * @return address of writing configuration
 */

static uint32_t const * address_of_page(uint16_t page_num)
{
    return param_fs_config.p_start_addr + (page_num * FS_MAX_WRITE_SIZE_WORDS /*NRF52 size is 1024*/);
}

static bool module_parameter_adjust_value(void)
{
    volatile int old_val;
    bool adjusted = false;
    if(((int)(m_module_parameter.idle_time) < 60) || ((int)(m_module_parameter.idle_time) > (60*60*24*7)))
    {
        adjusted = true;
        old_val = m_module_parameter.idle_time;
        if((int)(m_module_parameter.idle_time) < 60)m_module_parameter.idle_time = 60;
        else m_module_parameter.idle_time = (60*60*24*7);
        cPrintLog(CDBG_MAIN_LOG, "adjust idle_time %d to %d\n", old_val, m_module_parameter.idle_time);
    }

    if(((int)(m_module_parameter.beacon_interval) < 20) || ((int)(m_module_parameter.beacon_interval) > 10240))
    {
        adjusted = true;
        old_val = m_module_parameter.beacon_interval;
        if((int)(m_module_parameter.beacon_interval) < 20)m_module_parameter.beacon_interval = 20;
        else m_module_parameter.beacon_interval = 10240;
        cPrintLog(CDBG_MAIN_LOG, "adjust beacon_interval %d to %d\n", old_val, m_module_parameter.beacon_interval);
    }

    if(((int)(m_module_parameter.wifi_scan_retry_time_sec) < 0) || ((int)(m_module_parameter.wifi_scan_retry_time_sec) > CWIFI_SEC_RETRY_TIMEOUT_SEC_MAX))
    {
        adjusted = true;
        old_val = m_module_parameter.wifi_scan_retry_time_sec;
        if((int)(m_module_parameter.wifi_scan_retry_time_sec) < 0)m_module_parameter.wifi_scan_retry_time_sec = 0;  //0 is wifi off
        else m_module_parameter.wifi_scan_retry_time_sec = CWIFI_SEC_RETRY_TIMEOUT_SEC_MAX;
        cPrintLog(CDBG_MAIN_LOG, "adjust wifi_scan_retry_time_sec %d to %d\n", old_val, m_module_parameter.wifi_scan_retry_time_sec);
    }

    if(((int)(m_module_parameter.start_wait_time_for_board_control_attach_sec) > 10)) //use to sfm_boot_mode START_WAIT_TIME_FOR_BOARD_CONTROL_ATTACH_SEC
    {
        adjusted = true;
        old_val = m_module_parameter.start_wait_time_for_board_control_attach_sec;
        m_module_parameter.start_wait_time_for_board_control_attach_sec = 0;
        cPrintLog(CDBG_MAIN_LOG, "adjust start_wait_time_for_board_control_attach_sec %d to %d\n", old_val, m_module_parameter.start_wait_time_for_board_control_attach_sec);
    }

    if(((int)(m_module_parameter.start_wait_time_for_ctrl_mode_change_sec) > 1)) //use to sigfox dl enable
    {
        adjusted = true;
        old_val = m_module_parameter.start_wait_time_for_ctrl_mode_change_sec;
        m_module_parameter.start_wait_time_for_ctrl_mode_change_sec = 0;
        cPrintLog(CDBG_MAIN_LOG, "adjust start_wait_time_for_ctrl_mode_change_sec %d to %d\n", old_val, m_module_parameter.start_wait_time_for_ctrl_mode_change_sec);
    }

    if(((int)(m_module_parameter.gps_acquire_tracking_time_sec) < 30) || ((int)(m_module_parameter.gps_acquire_tracking_time_sec) > 7200))
    {
        adjusted = true;
        old_val = m_module_parameter.gps_acquire_tracking_time_sec;
        if((int)(m_module_parameter.gps_acquire_tracking_time_sec) < 30)m_module_parameter.gps_acquire_tracking_time_sec = 30;
        else m_module_parameter.gps_acquire_tracking_time_sec = 7200;
        cPrintLog(CDBG_MAIN_LOG, "adjust gps_acquire_tracking_time_sec %d to %d\n", old_val, m_module_parameter.gps_acquire_tracking_time_sec);
    }

    if(((int)(m_module_parameter.boot_nfc_unlock) < 0) || ((int)(m_module_parameter.boot_nfc_unlock) > 1))
    {
        adjusted = true;
        old_val = m_module_parameter.boot_nfc_unlock;
        if((int)(m_module_parameter.boot_nfc_unlock) < 0)m_module_parameter.boot_nfc_unlock = 0;
        else m_module_parameter.boot_nfc_unlock = 1;
        cPrintLog(CDBG_MAIN_LOG, "adjust boot_nfc_unlock %d to %d\n", old_val, m_module_parameter.boot_nfc_unlock);
    }

    if(((int)(m_module_parameter.fota_enable) < 0) || ((int)(m_module_parameter.fota_enable) > 1))
    {
        adjusted = true;
        old_val = m_module_parameter.fota_enable;
        if((int)(m_module_parameter.fota_enable) < 0)m_module_parameter.fota_enable = 0;
        else m_module_parameter.fota_enable = 1;
        cPrintLog(CDBG_MAIN_LOG, "adjust fota_enable %d to %d\n", old_val, m_module_parameter.fota_enable);
    }

    if(((int)(m_module_parameter.scenario_mode) < 0) || ((int)(m_module_parameter.scenario_mode) > MODULE_SCENARIO_MODE_MAX))
    {
        adjusted = true;
        old_val = m_module_parameter.scenario_mode;
        if((int)(m_module_parameter.scenario_mode) < 0)m_module_parameter.scenario_mode = 0;
        else m_module_parameter.scenario_mode = MODULE_SCENARIO_MODE_MAX;
        cPrintLog(CDBG_MAIN_LOG, "adjust scenario_mode %d to %d\n", old_val, m_module_parameter.scenario_mode);
    }

    if(((int)(m_module_parameter.magnetic_gpio_enable) < 0) || ((int)(m_module_parameter.magnetic_gpio_enable) > 1))
    {
        adjusted = true;
        old_val = m_module_parameter.magnetic_gpio_enable;
        if((int)(m_module_parameter.magnetic_gpio_enable) < 0)m_module_parameter.magnetic_gpio_enable = 0;
        else m_module_parameter.magnetic_gpio_enable = 1;
        cPrintLog(CDBG_MAIN_LOG, "adjust magnetic_gpio_enable %d to %d\n", old_val, m_module_parameter.magnetic_gpio_enable);
    }

    if(((int)(m_module_parameter.wkup_gpio_enable) < 0) || ((int)(m_module_parameter.wkup_gpio_enable) > 2))
    {
        adjusted = true;
        old_val = m_module_parameter.wkup_gpio_enable;
        if((int)(m_module_parameter.wkup_gpio_enable) < 0)m_module_parameter.wkup_gpio_enable = 0;
        else m_module_parameter.wkup_gpio_enable = 2;
        cPrintLog(CDBG_MAIN_LOG, "adjust wkup_gpio_enable %d to %d\n", old_val, m_module_parameter.wkup_gpio_enable);
    }

    if(((int)(m_module_parameter.wifi_testmode_enable) < 0) || ((int)(m_module_parameter.wifi_testmode_enable) > 1))
    {
        adjusted = true;
        old_val = m_module_parameter.wifi_testmode_enable;
        m_module_parameter.wifi_testmode_enable = 0;  //use to disable_battery_power_down
        cPrintLog(CDBG_MAIN_LOG, "adjust disable_battery_power_down %d to %d\n", old_val, m_module_parameter.wifi_testmode_enable);
    }

    if(((int)(m_module_parameter.sigfox_snek_testmode_enable) < 0) || ((int)(m_module_parameter.sigfox_snek_testmode_enable) > 1))
    {
        adjusted = true;
        old_val = m_module_parameter.sigfox_snek_testmode_enable;
        if((int)(m_module_parameter.sigfox_snek_testmode_enable) < 0)m_module_parameter.sigfox_snek_testmode_enable = 0;
        else m_module_parameter.sigfox_snek_testmode_enable = 1;
        cPrintLog(CDBG_MAIN_LOG, "adjust sigfox_snek_testmode_enable %d to %d\n", old_val, m_module_parameter.sigfox_snek_testmode_enable);
    }

    return adjusted;
}
bool module_parameter_fs_init(void)
{
    fs_ret_t fs_ret;
    if(m_module_parameter_fs_init_flag)
    {
        return true;
    }
    else
    {
        if(FS_MAX_WRITE_SIZE_WORDS < sizeof(m_module_parameter))
        {
            cPrintLog(CDBG_FLASH_ERR, "parameter too big more page size:%d, param size:%d\n", FS_MAX_WRITE_SIZE_WORDS, sizeof(m_module_parameter));
            APP_ERROR_CHECK(NRF_ERROR_NO_MEM);  //fs_config num_pages size over
        }
        fs_ret = fs_init();
        if(fs_ret == FS_SUCCESS)
        {
            m_module_parameter_fs_init_flag = true;
            cPrintLog(CDBG_FLASH_INFO, "fs_init OK! Addr:0x%p\n", param_fs_config.p_start_addr);
            return true;
        }
        else
        {
            cPrintLog(CDBG_FLASH_ERR, "fs_init Error! %d\n", fs_ret);
            m_module_parameter_fs_init_flag = false;
        }
    }
    return false;
}
static void module_parameter_early_read(void)
{
    if(!module_parameter_fs_init())
    {
        return;
    }
    memcpy(&m_module_parameter, param_fs_config.p_start_addr, sizeof(m_module_parameter));
}

bool module_parameter_get_bootmode(int *bootmode)
{
    if((m_module_parameter.magic_top == MODULE_PARAMETER_MAGIC_TOP && m_module_parameter.magic_bottom == MODULE_PARAMETER_MAGIC_BOTTOM))
    {
        if(bootmode)*bootmode = m_module_parameter.start_wait_time_for_board_control_attach_sec;
        return true;
    }
    return false;
}

bool module_parameter_erase_and_reset(void)  //for factory reset
{
    fs_ret_t fs_ret;
    if(!module_parameter_fs_init())
    {
        return false;
    }
    fs_ret = fs_erase(&param_fs_config, address_of_page(0), 1, NULL);

    if (fs_ret == FS_SUCCESS)
    {
        nrf_delay_ms(200);
        NVIC_SystemReset();
    }
    else
    {
        cPrintLog(CDBG_FLASH_ERR, "fs_erase error! %d\n",fs_ret);
        return false;
    }
    return true;
}

static void module_parameter_init(void)
{
    fs_ret_t fs_ret;
    bool parameter_rebuild_req = false;

    if(!module_parameter_fs_init())
    {
        return;
    }
    memcpy(&m_module_parameter, param_fs_config.p_start_addr, sizeof(m_module_parameter));

    if(!(m_module_parameter.magic_top == MODULE_PARAMETER_MAGIC_TOP && m_module_parameter.magic_bottom == MODULE_PARAMETER_MAGIC_BOTTOM))
    {
        parameter_rebuild_req = true;
    }
    else if(m_module_parameter.board_ID != m_cfg_board_type)
    {
        parameter_rebuild_req = true;
    }

    if(!parameter_rebuild_req)
    {
        cPrintLog(CDBG_FLASH_INFO, "module_parameter loaded!\n");
//        cDataDumpPrintOut(CDBG_FLASH_INFO, param_fs_config.p_start_addr, 64);
    }
    else
    {
        if(m_module_parameter.magic_top == 0)cPrintLog(CDBG_MAIN_LOG, "factory reset!\n");
        cPrintLog(CDBG_FLASH_INFO, "fs init to default!\n");
        module_parameter_default_init();
        // Erase one page (page 0).
        fs_ret = fs_erase(&param_fs_config, address_of_page(0), 1, NULL);
        if (fs_ret == FS_SUCCESS)
        {
            fs_ret = fs_store(&param_fs_config, param_fs_config.p_start_addr, (uint32_t const *)&m_module_parameter, (sizeof(m_module_parameter)/4)+1, NULL);
            if (fs_ret != FS_SUCCESS)
            {
                cPrintLog(CDBG_FLASH_ERR, "%s %d fs_store error! %d\n", __func__, __LINE__, fs_ret);
            }
        }
        else
        {
            cPrintLog(CDBG_FLASH_ERR, "%s %d fs_erase error! %d\n", __func__, __LINE__, fs_ret);
        }
        nrf_delay_ms(10);
    }
    module_parameter_adjust_value();
#ifdef CDEV_WIFI_MODULE
    cWifi_set_retry_time(m_module_parameter.wifi_scan_retry_time_sec);
#endif
    //for ID value cache
    memcpy(m_module_peripheral_ID.wifi_MAC_STA, m_module_parameter.wifi_MAC_STA, sizeof(m_module_peripheral_ID.wifi_MAC_STA));
    memcpy(m_module_peripheral_ID.sigfox_device_ID, m_module_parameter.sigfox_device_ID, sizeof(m_module_peripheral_ID.sigfox_device_ID));
    memcpy(m_module_peripheral_ID.sigfox_pac_code, m_module_parameter.sigfox_pac_code, sizeof(m_module_peripheral_ID.sigfox_pac_code));
}

static bool module_parameter_ID_value_update(void)
{
    bool ret = false;
    if((memcmp(m_module_peripheral_ID.wifi_MAC_STA, m_module_parameter.wifi_MAC_STA, sizeof(m_module_peripheral_ID.wifi_MAC_STA)) != 0)
        || (memcmp(m_module_peripheral_ID.sigfox_device_ID, m_module_parameter.sigfox_device_ID, sizeof(m_module_peripheral_ID.sigfox_device_ID)) != 0)
        || (memcmp(m_module_peripheral_ID.sigfox_pac_code, m_module_parameter.sigfox_pac_code, sizeof(m_module_peripheral_ID.sigfox_pac_code)) != 0)
    )
    {
        cPrintLog(CDBG_FLASH_INFO, "update ID Value!\n");
        //for ID value cache
        memcpy(m_module_parameter.wifi_MAC_STA, m_module_peripheral_ID.wifi_MAC_STA, sizeof(m_module_parameter.wifi_MAC_STA));
        memcpy(m_module_parameter.sigfox_device_ID, m_module_peripheral_ID.sigfox_device_ID, sizeof(m_module_parameter.sigfox_device_ID));
        memcpy(m_module_parameter.sigfox_pac_code, m_module_peripheral_ID.sigfox_pac_code, sizeof(m_module_parameter.sigfox_pac_code));

        m_module_parameter_update_req = true;
        module_parameter_check_update();
        ret = true;
    }
    return ret;
}

void module_parameter_check_update(void)
{
    fs_ret_t fs_ret;
    if(m_module_parameter_fs_init_flag && m_module_parameter_update_req)
    {
        nrf_delay_ms(1);
        cPrintLog(CDBG_FLASH_INFO, "%s %d excute parameter update!\n", __func__, __LINE__);
        fs_ret = fs_erase(&param_fs_config, address_of_page(0), 1, NULL);
        if (fs_ret == FS_SUCCESS)
        {
            fs_ret = fs_store(&param_fs_config, param_fs_config.p_start_addr, (uint32_t    const *)&m_module_parameter, (sizeof(m_module_parameter)/4)+1, NULL);
            if (fs_ret != FS_SUCCESS)
            {
                cPrintLog(CDBG_FLASH_ERR, "%s %d fs_store error! %d\n", __func__, __LINE__, fs_ret);
            }
        }
        else
        {
            cPrintLog(CDBG_FLASH_ERR, "%s %d fs_erase error! %d\n", __func__, __LINE__, fs_ret);
        }
        nrf_delay_ms(1);
        m_module_parameter_update_req = false;
    }
}

#if 0
/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = MSEC_TO_UNITS(m_module_parameter.beacon_interval, UNIT_0_625_MS);  //NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}
#else

static void get_ble_mac_address()
{
    uint32_t err_code;
    ble_gap_addr_t      addr;

    err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    m_module_peripheral_ID.ble_MAC[0] = addr.addr[5];
    m_module_peripheral_ID.ble_MAC[1] = addr.addr[4];
    m_module_peripheral_ID.ble_MAC[2] = addr.addr[3];
    m_module_peripheral_ID.ble_MAC[3] = addr.addr[2];
    m_module_peripheral_ID.ble_MAC[4] = addr.addr[1];
    m_module_peripheral_ID.ble_MAC[5] = addr.addr[0];

}


#define SCAN_INTERVAL           0x00A0                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x000A                                  /**< Timout when scanning. 0x0000 disables timeout. */
typedef void (*ble_scan_recv_handler_t)(ble_gap_evt_adv_report_t * p_adv_report);

static ble_gap_scan_params_t const m_ble_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,

    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};
bool m_ble_scan_run_flag;
ble_scan_recv_handler_t m_ble_scan_cb;

/**@brief Function to start scanning. */
void ble_scan_start(ble_scan_recv_handler_t recv_callback)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_ble_scan_params);
    APP_ERROR_CHECK(ret);
    m_ble_scan_cb = recv_callback;
    m_ble_scan_run_flag = true;
}
/**@brief Function to start scanning. */
void ble_scan_stop(void)
{
    (void)sd_ble_gap_scan_stop();
    m_ble_scan_cb = NULL;
    m_ble_scan_run_flag = false;
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
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


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
 //           err_code = bsp_indication_set(BSP_INDICATE_IDLE);
 //           APP_ERROR_CHECK(err_code);
            ble_connect_on = false;
            cPrintLog(CDBG_FCTRL_INFO, "BLE_GAP_EVT_DISCONNECTED.\r\n");
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
//            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            ble_connect_on = true;
            cPrintLog(CDBG_FCTRL_INFO, "BLE_GAP_EVT_CONNECTED.\r\n");
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            cPrintLog(CDBG_FCTRL_INFO, "GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            cPrintLog(CDBG_FCTRL_INFO, "GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif
        case BLE_GAP_EVT_ADV_REPORT:
            {
                ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
                ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
                if(m_ble_scan_cb)m_ble_scan_cb(p_adv_report);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            {
                ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
                {
                    m_ble_scan_run_flag = false;
                }
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    if(m_module_parameter.scenario_mode == MODULE_SCENARIO_MWC_DEMO)
    {
        advdata.name_type               = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance      = false;
        advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    }
    else
    {
        advdata.name_type               = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance      = true;
        advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    }

//    options.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    options.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = MSEC_TO_UNITS(m_module_parameter.beacon_interval, UNIT_0_625_MS);
    options.ble_adv_fast_timeout = 0; //APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for starting advertising.
 */
void advertising_start(bool start_flag, bool led_ctrl_flag)
{
#ifdef CDEV_BLE_ADVERTISING_ENABLE
    static bool started = false;
    uint32_t err_code;

    if(started != start_flag)
    {
        if(start_flag)
        {
            if(led_ctrl_flag)cfg_ble_led_control(true);
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            if(err_code != NRF_ERROR_CONN_COUNT && err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        else
        {
            if(led_ctrl_flag)cfg_ble_led_control(false);
            err_code = sd_ble_gap_adv_stop();
            if(err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        started = start_flag;
    }
#else
    (void)start_flag;
    (void)led_ctrl_flag;
    return;
#endif
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            cPrintLog(CDBG_BLE_DBG, "Connected to previously bonded device\r\n");
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_BONDED_PEER_CONNECTED

        case PM_EVT_CONN_SEC_START:
            break; // PM_EVT_CONN_SEC_START

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            cPrintLog(CDBG_BLE_DBG, "Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                                 ble_conn_state_role(p_evt->conn_handle),
                                 p_evt->conn_handle,
                                 p_evt->params.conn_sec_succeeded.procedure);
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_CONN_SEC_SUCCEEDED

        case PM_EVT_CONN_SEC_FAILED:
        {
            /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
             *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
             *  be restarted until the link is disconnected and reconnected. Sometimes it is
             *  impossible, to secure the link, or the peer device does not support it. How to
             *  handle this error is highly application dependent. */
 
        } break; // PM_EVT_CONN_SEC_FAILED

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break; // PM_EVT_CONN_SEC_CONFIG_REQ

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_STORAGE_FULL

        case PM_EVT_ERROR_UNEXPECTED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
            break; // PM_EVT_ERROR_UNEXPECTED

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break; // PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            // Assert.
            APP_ERROR_CHECK_BOOL(false);
            break; // PM_EVT_PEER_DATA_UPDATE_FAILED

        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break; // PM_EVT_PEER_DELETE_SUCCEEDED

        case PM_EVT_PEER_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
            break; // PM_EVT_PEER_DELETE_FAILED

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(true, true);
            break; // PM_EVT_PEERS_DELETE_SUCCEEDED

        case PM_EVT_PEERS_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
            break; // PM_EVT_PEERS_DELETE_FAILED

        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break; // PM_EVT_LOCAL_DB_CACHE_APPLIED

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break; // PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
//    bsp_btn_ble_on_ble_evt(p_ble_evt);
#ifdef CDEV_NUS_MODULE
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_nus_evt(&m_nus, p_ble_evt);
#endif
#ifdef CDEV_BATT_CHECK_MODULE
//    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
#endif
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    if(m_module_parameter.fota_enable)
    {
        ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    }
    /*YOUR_JOB add calls to _on_ble_evt functions from each service your application is using
       ble_xxs_on_ble_evt(&m_xxs, p_ble_evt);
       ble_yys_on_ble_evt(&m_yys, p_ble_evt);
     */
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */

static void sys_evt_dispatch(uint32_t event)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(event);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(event);

}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC_250_PPM;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
#ifdef CDEV_NUS_MODULE
    ble_enable_params.common_enable_params.vs_uuid_count   = 2;
#endif
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    m_softdevice_init_flag = true;

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

//    if (erase_bonds)
//    {
//        err_code = pm_peers_delete();
//        APP_ERROR_CHECK(err_code);
//    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


#define APP_BEACON_INFO_LENGTH          0x11
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH];

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    if(m_module_parameter.scenario_mode == MODULE_SCENARIO_MWC_DEMO)
    {
        unsigned int index = 0;

        memset(m_beacon_info, 0, sizeof(m_beacon_info));
        strcpy((char*)&m_beacon_info[index], "SFM");
        index += 3;
        memcpy(&m_beacon_info[index], (m_nus_service_parameter.wifi_data), CWIFI_BSSID_SIZE);
        index += CWIFI_BSSID_SIZE;
        memcpy(&m_beacon_info[index], (m_nus_service_parameter.ap_key), strlen((const char *)m_nus_service_parameter.ap_key));
        index += strlen((const char *)m_nus_service_parameter.ap_key);
        
        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)m_beacon_info,
                                              index);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2) 
        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)ASSETTRACKER_NAME,
                                              strlen(ASSETTRACKER_NAME));
#elif (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI)
        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)MINI_ASSETTRACKER_NAME,
                                              strlen(MINI_ASSETTRACKER_NAME));
#elif (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
        {
            char device_name[32];
            sprintf(device_name, "%s_V%s", M3_ASSETTRACKER_NAME /*ASSETTRACKER_NAME*/, m_cfg_sw_ver);
            err_code = sd_ble_gap_device_name_set(&sec_mode,
                                                  (const uint8_t *)device_name,
                                                  strlen(device_name));
        }
#else
        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)DEVICE_NAME,
                                              strlen(DEVICE_NAME));
#endif
        APP_ERROR_CHECK(err_code);
    }

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
   static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
   {
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
   }*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_dfu_init_t dfus_init;
//    ble_gap_addr_t      addr;

#ifdef CDEV_NUS_MODULE
    ble_nus_init_t nus_init;
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
#endif
   //   Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler                               = ble_dfu_evt_handler;
    dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
    dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;

    if(m_module_parameter.fota_enable)
    {
        if(m_cfg_available_bootloader_detected)
        {
            cPrintLog(CDBG_BLE_ERR, "bootloader detected!\n");
        }
        else
        {
            cPrintLog(CDBG_BLE_ERR, "=== warning! bootloader not detected! ===\n");
        }
        err_code = ble_dfu_init(&m_dfus, &dfus_init);
        APP_ERROR_CHECK(err_code);
    }

/*
    err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    m_module_peripheral_ID.ble_MAC[0] = addr.addr[5];
    m_module_peripheral_ID.ble_MAC[1] = addr.addr[4];
    m_module_peripheral_ID.ble_MAC[2] = addr.addr[3];
    m_module_peripheral_ID.ble_MAC[3] = addr.addr[2];
    m_module_peripheral_ID.ble_MAC[4] = addr.addr[1];
    m_module_peripheral_ID.ble_MAC[5] = addr.addr[0];
    */
    cPrintLog(CDBG_FCTRL_DBG, "MAC ADDR:");
    cDataDumpPrintOut(CDBG_FCTRL_DBG, m_module_peripheral_ID.ble_MAC, BLE_GAP_ADDR_LEN);

    /* YOUR_JOB: Add code to initialize the services used by the application.
       uint32_t                           err_code;
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code;
    if(m_softdevice_init_flag)
    {
        err_code = sd_app_evt_wait();
        if(m_softdevice_init_flag)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    else
    {
        __WFE();
    }
}


/**
 * @brief Function for starting idle timer.
 */
void main_timer_idle_start(void)
{
    uint32_t      err_code;

    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(1000*m_module_parameter.idle_time, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for starting main schedule timer.
 *
*/
void main_timer_schedule_start(void)
{
    uint32_t      err_code;

    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(APP_MAIN_SCHEDULE_MS, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for restarting main schedule at idle state
 *
*/
void main_timer_schedule_restart_check_idle(void)
{
    if(main_schedule_state_is_idle())
    {
        if(m_module_parameter.scenario_mode == MODULE_SCENARIO_IHERE_MINI)
        {
            ihere_mini_current_schedule_start();
        }
        else
        {
            main_timer_schedule_stop();
            main_timer_schedule_start();
            if(m_module_parameter.scenario_mode == MODULE_SCENARIO_MWC_DEMO)
            {
                advertising_start(false, true);
            }
        }
    }
}

/**
 * @brief Function for starting variable timer.
 */
void main_timer_variable_schedule_start(unsigned int schedule_time_sec)
{
    uint32_t      err_code;

    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(1000*schedule_time_sec, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for stop  main schedule timer.
 */
void main_timer_schedule_stop(void)
{
    uint32_t      err_code;
    err_code = app_timer_stop(m_main_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting main state in main scheduler.
 *
 * @param[in] state  set the next state in main scheduler.
 *                         
 */

void main_set_module_state(module_mode_t state)
{
    m_init_excute = true; /*for run initial*/
    m_module_ready_wait_timeout_tick = 0; /*for timeout check*/
    m_module_mode = state;
}

/**@brief Function for indicating whether the current state is idle or not.
 *
 * @return true if it is idle, nor false
 *                         
 */

bool main_schedule_state_is_idle(void)
{
    if(m_module_mode == IDLE)
        return true;
    else
        return false;
}

void main_wkup_det_callback(void)
{
    main_wkup_push_detected = true;
    cTBC_event_noti("WKUP_KEY");
    if(!main_wakeup_interrupt && main_schedule_state_is_idle())
    {
        main_wakeup_reason = main_wakeup_reason_key_event;
        main_wakeup_interrupt = true;
    }
}

void main_magnet_attach_callback(void)
{
    main_magnet_detected = true;
    cTBC_event_noti("MAGNET_ATT");
    if(!main_wakeup_interrupt && main_schedule_state_is_idle())
    {
        main_wakeup_reason = main_wakeup_reason_magnetic_event;
        main_wakeup_interrupt = true;
    }
}

#ifdef PIN_DEF_BUTTON  //CDEV_BOARD_M3
void main_button_short_press_callback(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "pwr key short\n");
    cTBC_event_noti("BUTTON_KEY");
    main_button_detected = true;
    if(!main_wakeup_interrupt && main_schedule_state_is_idle())
    {
        cPrintLog(CDBG_FCTRL_INFO, "Wakeup occurred\n");
        main_wakeup_reason = main_wakeup_reason_key_event;
        main_wakeup_interrupt = true;
    }
}

void main_button_long_press_callback(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "request power off\n");
    main_powerdown_request = true;
}
#endif

/**
 * @brief Callback function for handling main state  from main schedule timer.
 * @details manage main scheduling in normal demo
 */

static void main_schedule_timeout_handler_asset_tracker(void * p_context)
{
    UNUSED_PARAMETER(p_context);
#ifdef CDEV_GPS_MODULE
    bool send_gps_data_req = false;
    uint8_t gps_send_data[SIGFOX_SEND_PAYLOAD_SIZE];
    int get_nmea_result = 0;
    uint8_t *nmea_Buf;
    static int work_mode = GPS_START;  //module_work_e
#endif
    static bool old_ble_led = false;
    if(m_module_mode != m_module_mode_old)
    {
        cPrintLog(CDBG_FCTRL_DBG, "main state changed %d to %d\n", m_module_mode_old, m_module_mode);
        m_module_mode_old = m_module_mode;
    }

    main_schedule_tick++;
    switch(m_module_mode)
    {
        case ACC:
            if(bma250_get_state() == NONE_ACC)
            {
                cPrintLog(CDBG_FCTRL_INFO, "%s %d ACC MODULE started\n",  __func__, __LINE__);
                bma250_set_state(SET_S);
                cfg_bma250_timers_start();
            }
            else if(bma250_get_state() == EXIT_ACC)
            {
                cfg_bma250_timers_stop();
//                bma250_set_state(NONE_ACC);
            }
            main_set_module_state(MAIN_SCENARIO_LOOP);
            break;

        case MAIN_SCENARIO_LOOP:
            cPrintLog(CDBG_FCTRL_INFO, "==MAIN_SCENARIO_LOOP %u==\n", main_schedule_tick);
            main_set_module_state(BATTERY_CHECK);
            cfg_board_common_power_control(module_comm_pwr_common, true);
            break;

        case BATTERY_CHECK:
#ifdef PIN_DEF_BATTERY_ADC_INPUT
            ++m_module_ready_wait_timeout_tick;
            if(m_init_excute)
            {
                adc_configure();
                cfg_bas_timer_start();
                m_init_excute = false;
            }
            else
            {
                if(m_module_ready_wait_timeout_tick > (APP_MAIN_SCHEDULE_HZ/2))  //wait 500ms
                {
                    bool log_once_flag = false;
                    uint16_t avg_batt_lvl_in_milli_volts;

                    if(m_module_ready_wait_timeout_tick == (APP_MAIN_SCHEDULE_HZ * 2))log_once_flag=true;
                    m_nus_service_parameter.battery_volt[0] = avg_report_volts;
                    m_nus_service_parameter.module='B';
                    avg_batt_lvl_in_milli_volts = get_avg_batt_lvl_in_milli_volts();
                    if(3000 <= avg_batt_lvl_in_milli_volts && 5200 >= avg_batt_lvl_in_milli_volts)
                    {
                        if(m_module_parameter.wifi_testmode_enable /*disable_battery_power_down*/)cPrintLog(CDBG_MAIN_LOG, "Battery Pwr Off Disabled\n");
                        if(!m_module_parameter.wifi_testmode_enable /*disable_battery_power_down*/ && !cTBC_check_host_connected() 
#if (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
                            && avg_batt_lvl_in_milli_volts < 3100
#else
                            && avg_batt_lvl_in_milli_volts < 3500
#endif
                        )  //low battery
                        {
                            if(log_once_flag)cPrintLog(CDBG_MAIN_LOG, "Battery Low:%d\n", avg_batt_lvl_in_milli_volts);
                            main_powerdown_request = true;
                        }
                        else if(!m_module_parameter.wifi_testmode_enable /*disable_battery_power_down*/ 
#if (CDEV_BOARD_TYPE == CDEV_BOARD_M3)
                            && avg_batt_lvl_in_milli_volts < 3300
#else
                            && avg_batt_lvl_in_milli_volts < 3600
#endif
                        )  //battery warning
                        {
                            if(log_once_flag)cPrintLog(CDBG_MAIN_LOG, "Battery Warning:%d\n", avg_batt_lvl_in_milli_volts);
                            if(m_module_ready_wait_timeout_tick > (APP_MAIN_SCHEDULE_HZ * 4))  //wait more 2 sec for warning noti
                            {
                                main_set_module_state(TMP);   //battery value is enough
                            }
                            else
                            {
                                if(((m_module_ready_wait_timeout_tick % APP_MAIN_SCHEDULE_HZ) == 0))
                                {
                                    cfg_ble_led_control(false);
                                }
                                else if(((m_module_ready_wait_timeout_tick % APP_MAIN_SCHEDULE_HZ) == 2))
                                {
                                    cfg_ble_led_control(true);
                                }
                            }
                        }
                        else
                        {
                            main_set_module_state(TMP);   //battery value is enough
                        }
                    }
                    else
                    {
                        if(log_once_flag)cPrintLog(CDBG_MAIN_LOG, "ADC Not Available:%d\n", avg_batt_lvl_in_milli_volts);
                        main_set_module_state(TMP);  //battery value is not available
                    }
                }
            }
#else
            main_set_module_state(TMP);
#endif
            break;

        case TMP:
            if(tmp102_get_state() == NONE_TMP)
            {
                cTBC_write_state_noti("Temperature");
                cPrintLog(CDBG_FCTRL_INFO, "%s %d TMP MODULE started\n",  __func__, __LINE__);
                tmp102_set_state(TMP_SET_S);
                cfg_tmp102_timers_start();
            }
            else if(tmp102_get_state() == EXIT_TMP)
            {
                cfg_tmp102_timers_stop();
                tmp102_set_state(NONE_TMP);
                main_set_module_state(GPS);
//                nus_send_data('T');
            }
            break;
        case BLE:
            cTBC_write_state_noti("BleAdvertising");
            cPrintLog(CDBG_BLE_INFO, "BLE MODULE started\r\n");
//            advertising_start(true, true);
            if(cfg_is_3colorled_contorl())
            {
                cfg_ble_led_control(false);
            }
            cPrintLog(CDBG_FCTRL_INFO, "%s %d BLE MODULE started\n", __func__, __LINE__);
#ifdef CDEV_NUS_MODULE
            m_nus_service_parameter.magnet_event = '0';
            m_nus_service_parameter.accellometer_event = '0';
#endif
            main_wakeup_reason = main_wakeup_reason_normal;  //clear event reason
            main_timer_schedule_stop();
            main_timer_idle_start();
            main_set_module_state(IDLE);
            cfg_board_common_power_control(module_comm_pwr_common, false);
            break;
        case GPS:
#ifdef CDEV_GPS_MODULE
            memset(gps_send_data, 0, sizeof(gps_send_data));

            if(work_mode == GPS_START)
            {
#ifdef PIN_DEF_GPS_BKUP_CTRL_WITH_PULLUP  //GPS_BKUP_CTRL
                cfg_board_GPS_BKUP_ctrl(true);  //GPS_BKUP_CTRL
                main_GPS_BKUP_time_out = (2*60*60)/*COLD_START_TIME*/;  //GPS_BKUP_CTRL
#endif
                if(cfg_is_3colorled_contorl())
                {
                    old_ble_led = cfg_ble_led_control(false);
                    cfg_wkup_output_control(true);
                }
                if(cGps_status_available() == CGPS_Result_OK)
                {
                    cTBC_write_state_noti("GPS");
                    cPrintLog(CDBG_FCTRL_INFO, "%s %d GPS MODULE started  \n", __func__, __LINE__);
                    cGps_nmea_acquire_request();                  
                    work_mode = GPS_WORK; //wait scan
                }
                else
                {
                    work_mode = GPS_END;
                }
            }
            else if(work_mode == GPS_WORK)
            {
                get_nmea_result = cGps_acquire_tracking_check();
                if(get_nmea_result == CGPS_Result_OK)
                {
                    work_mode = GPS_END;
                }
                else  if(get_nmea_result ==  CGPS_Result_Busy)
                {
                    ;
                }
                else
                {
                    if(!cGps_bus_busy_check())
                    {
                        work_mode = GPS_START;
                        if(cfg_is_3colorled_contorl())
                        {
                            cfg_ble_led_control(old_ble_led);
                            cfg_wkup_output_control(false);
                        }
                        main_set_module_state(WIFI);
                        nus_send_data('G');
                    }
                }
            }
            else if(work_mode == GPS_END)
            {
                get_nmea_result = cGps_nmea_get_bufPtr(&nmea_Buf);
                if(get_nmea_result == CGPS_Result_OK)
                {
                    memcpy(gps_send_data, nmea_Buf, sizeof(gps_send_data));
                    send_gps_data_req = true;
                }
                if(send_gps_data_req)
                {
                    cfg_bin_2_hexadecimal(gps_send_data, SIGFOX_SEND_PAYLOAD_SIZE, (char *)frame_data);  //set sigfox payload
                    cPrintLog(CDBG_FCTRL_INFO, "[GPS] %d send request sigfox! frame_data:[%s]  \n", __LINE__, frame_data);
                    send_gps_data_req = false;
                    work_mode = GPS_START;
#ifdef CDEV_NUS_MODULE
                    memcpy(m_nus_service_parameter.gps_data, gps_send_data, sizeof(m_nus_service_parameter.gps_data));
//                    memcpy(m_nus_service_parameter.temperature_data, &gps_send_data[9], sizeof(m_nus_service_parameter.temperature_data));
                    m_nus_service_parameter.module='G';
#endif
                    if(cfg_is_3colorled_contorl())
                    {
                        cfg_ble_led_control(old_ble_led);
                        cfg_wkup_output_control(false);
                    }
#ifdef TEST_SIGFOX_CURRENT_CONSUMPTION
                    if(!cGps_bus_busy_check())
                    {
                        work_mode = GPS_START;
                        if(cfg_is_3colorled_contorl())
                        {
                            cfg_ble_led_control(old_ble_led);
                            cfg_wkup_output_control(false);
                        }
                        main_set_module_state(WIFI);
                    }
#else
                    main_set_module_state(SIGFOX);
#endif
                }
                else
                {
                    if(!cGps_bus_busy_check())
                    {
                        work_mode = GPS_START;
                        if(cfg_is_3colorled_contorl())
                        {
                            cfg_ble_led_control(old_ble_led);
                            cfg_wkup_output_control(false);
                        }
                        main_set_module_state(WIFI);
                    }
                }
                nus_send_data('G');
            }
#else
            cPrintLog(CDBG_FCTRL_INFO, "%s %d CDEV_GPS_MODULE Not Defined!\n", __func__, __LINE__);
            if(cfg_is_3colorled_contorl())
            {
                cfg_ble_led_control(old_ble_led);
                cfg_wkup_output_control(false);
            }
            main_set_module_state(WIFI);
#endif
            break;
        case WIFI:
            {
                bool send_data_req = false;
                uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE] = {0,};
#ifdef CDEV_WIFI_MODULE
                int get_bssid_result;
                uint8_t *bssidBuf;
                int wifi_result;

                ++m_module_ready_wait_timeout_tick;
                if(m_init_excute)
                {
                    m_module_peripheral_data.sigfixdata_wifi_flag = 0;
#ifdef CDEV_NUS_MODULE
                    memset(m_nus_service_parameter.wifi_data, 0, sizeof(m_nus_service_parameter.wifi_data));
                    memset(m_nus_service_parameter.wifi_rssi, 0, sizeof(m_nus_service_parameter.wifi_rssi));
#endif
                    wifi_result = cWifi_ap_scan_req();
                    if(wifi_result == CWIFI_Result_OK)
                    {
                        cTBC_write_state_noti("WifiScan");
                        cPrintLog(CDBG_FCTRL_INFO, "%s %d WIFI MODULE started!\n", __func__, __LINE__);
                        if(cfg_is_3colorled_contorl())
                        {
                            old_ble_led = cfg_ble_led_control(true);
                            cfg_wkup_output_control(true);
                        }
                        m_init_excute = false;
                    }
                    else
                    {
                        cPrintLog(CDBG_FCTRL_INFO, "%s %d Not Availalble Wifi Module! send NUll data!\n", __func__, __LINE__);
                        send_data_req = true;
                    }
                }
                else
                {
                    if(!cWifi_is_scan_state() && !cWifi_bus_busy_check())  //wait scan
                    {
                        if(cfg_is_3colorled_contorl())
                        {
                            cfg_ble_led_control(old_ble_led);
                            cfg_wkup_output_control(false);
                        }
                        get_bssid_result = cWifi_get_BSSIDs_bufPtr(&bssidBuf);
                        cPrintLog(CDBG_FCTRL_INFO, "%s %d WIFI MODULE end! result:%d BSSID:", __func__, __LINE__, get_bssid_result);
                        cDataDumpPrintOut(CDBG_FCTRL_INFO, bssidBuf, (CWIFI_BSSID_CNT*CWIFI_BSSID_SIZE));
                        if(get_bssid_result == CWIFI_Result_OK)
                        {
                            m_module_peripheral_data.sigfixdata_wifi_flag = 1;
                            memcpy(send_data, bssidBuf, sizeof(send_data));
#ifdef CDEV_NUS_MODULE
                            {
                                int32_t *rssi;
                                cWifi_get_scan_result(NULL, NULL, &rssi, NULL);
                                memcpy(m_nus_service_parameter.wifi_data, bssidBuf, sizeof(m_nus_service_parameter.wifi_data));
                                m_nus_service_parameter.wifi_rssi[0] = (int8_t)rssi[0];
                                m_nus_service_parameter.wifi_rssi[1] = (int8_t)rssi[1];
//                                cPrintLog(CDBG_FCTRL_INFO, "==============WIFI RSSI %d %d", m_nus_service_parameter.wifi_rssi[0], m_nus_service_parameter.wifi_rssi[1]);
                                m_nus_service_parameter.module = 'W';
                                nus_send_data('W');
                            }
#endif
                        }
                        else if(get_bssid_result == CWIFI_Result_NoData)
                        {
                            cPrintLog(CDBG_FCTRL_INFO, "%s %d WIFI MODULE NoData! send NUll data!\n", __func__, __LINE__);
                        }
                        else
                        {
                            cPrintLog(CDBG_FCTRL_INFO, "%s %d Not Availalble Wifi Module! send NUll data!\n", __func__, __LINE__);
                        }
                        send_data_req = true;
                    }
                }                    
#else
                m_module_peripheral_data.sigfixdata_wifi_flag = 0;
#if (CDEV_MODULE_TYPE == CDEV_MODULE_SFM50R)
                send_data[0] = avg_report_volts;
                send_data[1] = tmp102a_sigfox;
                send_data[2] = tmp102b_sigfox;

                cPrintLog(CDBG_FCTRL_INFO, "%s %d CDEV_WIFI_MODULE Not Defined!\n", __func__, __LINE__);
                cPrintLog(CDBG_FCTRL_INFO, "%s %d Voltage[%d][0x%02x]-100mV / Tmp102[%d.%d][0x%02x.0x%02x]  !\n", __func__, __LINE__, 
                    avg_report_volts, avg_report_volts, tmp102a_sigfox,tmp102b_sigfox, tmp102a_sigfox,tmp102b_sigfox);
#else /* CDEV_MODULE_SFM50R */
                cPrintLog(CDBG_FCTRL_INFO, "%s %d CDEV_WIFI_MODULE Not Defined! send NUll data!\n", __func__, __LINE__);

#ifdef FEATURE_ONSEMI_IHERE_DEMO_TEST
                send_data[8] = avg_report_volts;
                send_data[10] = tmp102a_sigfox;
                send_data[11] = tmp102b_sigfox;
                cPrintLog(CDBG_FCTRL_INFO, "%s %d Voltage[%d][0x%02x]-100mV / Tmp102[%d.%d][0x%02x.0x%02x]  !\n", __func__, __LINE__, 
                    avg_report_volts, avg_report_volts, tmp102a_sigfox,tmp102b_sigfox, tmp102a_sigfox,tmp102b_sigfox);

#endif /* FEATURE_ONSEMI_IHERE_DEMO_TEST */

#endif /* CDEV_MODULE_SFM50R */
                send_data_req = true;
#endif /* CDEV_WIFI_MODULE */
                if(send_data_req)
                {
                    memcpy(m_module_peripheral_data.sigfixdata_wifi, send_data, sizeof(m_module_peripheral_data.sigfixdata_wifi));
                    cfg_bin_2_hexadecimal(send_data, SIGFOX_SEND_PAYLOAD_SIZE, (char *)frame_data);  //set sigfox payload
                    cPrintLog(CDBG_FCTRL_INFO, "%s %d send request sigfox! data:%s\n", __func__, __LINE__, frame_data);
                    main_set_module_state(SIGFOX);
                }
            }
            break;

        case SIGFOX:
            if(sigfox_get_state() == SETUP_S)
            {
                if(m_init_excute)
                {
                    cTBC_write_state_noti("Sigfox");
                    cPrintLog(CDBG_FCTRL_INFO, "%s %d SIGFOX MODULE started\n", __func__, __LINE__);
                    cfg_sigfox_timers_start();

                    m_init_excute = false;
                }
            }
            else if(sigfox_check_exit_excute())
            {
                main_set_module_state(BLE);
                cfg_sigfox_timers_stop();
                sigfox_set_state(SETUP_S);
                nus_send_data('T');
                nus_send_data('B');
            }
            break;

        case IDLE:
            cPrintLog(CDBG_FCTRL_INFO, "IDLE MODULE started\r\n");
//            advertising_start(false, true);

            main_timer_schedule_stop();
            main_timer_schedule_start();
//            m_module_mode = ACC;

            main_set_module_state(MAIN_SCENARIO_LOOP);
            if(cfg_is_3colorled_contorl())
            {
                cfg_ble_led_control(true);
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Callback function for handling main state  from main schedule timer.
 * @details manage main scheduling in MWC demo
 */

static void main_schedule_timeout_handler_MWC_demo(void * p_context)
{
    static bool old_ble_led = false;
    uint8_t *p_down_link_data;
    uint32_t down_link_data_size;

    main_schedule_tick++;
    switch(m_module_mode)
    {
        case MAIN_SCENARIO_LOOP:
            cPrintLog(CDBG_FCTRL_INFO, "==MAIN_SCENARIO_LOOP %u==\n", main_schedule_tick);
            main_set_module_state(WIFI);
            cfg_board_common_power_control(module_comm_pwr_common, true);
            break;

        case BLE:
            main_timer_schedule_stop();
            if(m_module_parameter.scenario_mode == MODULE_SCENARIO_MWC_DEMO)
            {
                advertising_start(false, true);
                sigfox_get_ap_key(m_nus_service_parameter.ap_key);
                gap_params_init();
                advertising_init();
                advertising_start(true, true);
            }
            main_timer_idle_start();
            main_set_module_state(IDLE);
            cfg_board_common_power_control(module_comm_pwr_common, false);
            break;

        case WIFI:
            {
                bool send_data_req = false;
                uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE];
                memset(send_data, 0, sizeof(send_data));
                send_data[0] = 'C'; //0x43
                send_data[1] = 0x01; //wifi password request
#ifdef CDEV_WIFI_MODULE
                int get_bssid_result;
                uint8_t *bssidBuf;
                int wifi_result;

                ++m_module_ready_wait_timeout_tick;
                if(m_init_excute)
                {
                    m_module_peripheral_data.sigfixdata_wifi_flag = 0;
                    wifi_result = cWifi_ap_get_available_first_BSSID("NECTAR");
                    if(wifi_result == CWIFI_Result_OK)
                    {
                        cTBC_write_state_noti("WifiScan");
                        cPrintLog(CDBG_FCTRL_INFO, "%s %d WIFI MODULE started!\n", __func__, __LINE__);
                        if(cfg_is_3colorled_contorl())
                        {
                            old_ble_led = cfg_ble_led_control(true);
                            cfg_wkup_output_control(true);
                        }
                        m_init_excute = false;
                    }
                    else
                    {
                        cPrintLog(CDBG_FCTRL_INFO, "%s %d Not Availalble Wifi Module! send NUll data!\n", __func__, __LINE__);
                        send_data_req = true;
                    }
                }
                else
                {
                    if(!cWifi_is_scan_state() && !cWifi_bus_busy_check())  //wait scan
                    {
                        if(cfg_is_3colorled_contorl())
                        {
                            cfg_ble_led_control(old_ble_led);
                            cfg_wkup_output_control(false);
                        }
                        get_bssid_result = cWifi_get_BSSIDs_bufPtr(&bssidBuf);
                        cPrintLog(CDBG_FCTRL_INFO, "%s %d WIFI MODULE end! result:%d BSSID:", __func__, __LINE__, get_bssid_result);
                        cDataDumpPrintOut(CDBG_FCTRL_INFO, bssidBuf, (CWIFI_BSSID_CNT*CWIFI_BSSID_SIZE));
                        if(get_bssid_result == CWIFI_Result_OK)
                        {
                            m_module_peripheral_data.sigfixdata_wifi_flag = 1;
                            memcpy(&send_data[2], bssidBuf, 6);
#ifdef CDEV_NUS_MODULE
                            memset(m_nus_service_parameter.wifi_data,0,12);
                            memcpy(m_nus_service_parameter.wifi_data, bssidBuf, CWIFI_BSSID_SIZE);
#endif
                        }
                        else if(get_bssid_result == CWIFI_Result_Busy)
                        {
                            cPrintLog(CDBG_FCTRL_INFO, "%s %d WIFI MODULE Busy! send NUll data!\n", __func__, __LINE__);
                        }
                        else if(get_bssid_result == CWIFI_Result_NoData)
                        {
                            cPrintLog(CDBG_FCTRL_INFO, "%s %d WIFI MODULE NoData! send NUll data!\n", __func__, __LINE__);
                        }
                        else
                        {
                            cPrintLog(CDBG_FCTRL_INFO, "%s %d Not Availalble Wifi Module! send NUll data!\n", __func__, __LINE__);
                        }
                        send_data_req = true;
                    }
                }                    
#else
                m_module_peripheral_data.sigfixdata_wifi_flag = 0;
                cPrintLog(CDBG_FCTRL_INFO, "%s %d CDEV_WIFI_MODULE Not Defined! send NUll data!\n", __func__, __LINE__);
                send_data_req = true;
#endif
                if(send_data_req)
                {
                    memcpy(m_module_peripheral_data.sigfixdata_wifi, send_data, sizeof(m_module_peripheral_data.sigfixdata_wifi));
                    cfg_bin_2_hexadecimal(send_data, SIGFOX_SEND_PAYLOAD_SIZE, (char *)frame_data);  //set sigfox payload
                    cPrintLog(CDBG_FCTRL_INFO, "%s %d send request sigfox! data:%s\n", __func__, __LINE__, frame_data);
                    main_set_module_state(SIGFOX);
                }
            }
            break;

        case SIGFOX:
            if(sigfox_get_state() == SETUP_S)
            {
                if(m_init_excute)
                {
                    cTBC_write_state_noti("Sigfox");
                    cPrintLog(CDBG_FCTRL_INFO, "%s %d SIGFOX MODULE started\n", __func__, __LINE__);
                    if(cfg_is_3colorled_contorl())
                    {
                        old_ble_led = cfg_ble_led_control(false);
                    }
                    memset(m_nus_service_parameter.ap_key, 0, sizeof(m_nus_service_parameter.ap_key));
                    cfg_sigfox_timers_start();
                    m_init_excute = false;
                }
            }
            else if(sigfox_check_exit_excute())
            {
                if(cfg_is_3colorled_contorl())
                {
                    cfg_ble_led_control(old_ble_led);
                }
                main_set_module_state(BLE);
                cfg_sigfox_timers_stop();
                sigfox_set_state(SETUP_S);
                p_down_link_data = cfg_sigfox_get_downlink_ptr(&down_link_data_size);
                if(down_link_data_size > 0)
                {
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX downlink data:%s, size:%d\n", p_down_link_data, down_link_data_size);
                    (void)p_down_link_data;
                    //todo ble advertising payload p_down_link_data
                    //advertising_start(false);
                    //advertising_start(true);
                }
            }
            break;

        case IDLE:
            main_timer_schedule_stop();
            main_timer_schedule_start();
            main_set_module_state(MAIN_SCENARIO_LOOP);
            break;
        default:
            break;
    }
}

static void main_bypass_enter_CB(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "call %s\n", __func__);
    advertising_start(false, true);
#ifdef CDEV_GPS_MODULE
    cGps_power_control(false, false);
#endif
}

static void main_bypass_exit_CB(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "call %s\n", __func__);
    advertising_start(true, true);
#ifdef CDEV_GPS_MODULE
    cGps_power_control(true, false);
#endif
}

/**
 * @brief function for creating main schedule timer
 */

static void main_timer_create()
{
    app_timer_timeout_handler_t timeout_handler;
    volatile uint32_t      err_code;

    if(m_module_parameter.scenario_mode == MODULE_SCENARIO_ASSET_TRACKER)
    {
        cPrintLog(CDBG_FCTRL_INFO, "start mode : MODULE_SCENARIO_ASSET_TRACKER\n");
        timeout_handler = main_schedule_timeout_handler_asset_tracker;
    }
    else if(m_module_parameter.scenario_mode == MODULE_SCENARIO_MWC_DEMO)
    {
        cPrintLog(CDBG_FCTRL_INFO, "start mode : MODULE_SCENARIO_MWC_DEMO\n");
        timeout_handler = main_schedule_timeout_handler_MWC_demo;
    }
    else if(m_module_parameter.scenario_mode == MODULE_SCENARIO_IHERE_MINI)
    {
        cPrintLog(CDBG_FCTRL_INFO, "start mode : MODULE_SCENARIO_IHERE_MINI\n");
        timeout_handler = main_schedule_timeout_handler_ihere_mini;
    }
    else
    {
        cPrintLog(CDBG_FCTRL_ERR, "%s invalid scenario %d \n", __func__, m_module_parameter.scenario_mode);
        cPrintLog(CDBG_FCTRL_ERR, "goto ASSET TRACKER! \n");
        timeout_handler = main_schedule_timeout_handler_asset_tracker;
    }
    err_code = app_timer_create(&m_main_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief function for prepare power down
 */
static void main_prepare_power_down(void)
{
    cPrintLog(CDBG_MAIN_LOG, "prepare power down\n");
    if(!m_cfg_i2c_master_init_flag)
        cfg_i2c_master_init();
    nrf_delay_ms(1);
    bma250_read_chip();

    bma250_req_suppend_mode();
    nrf_delay_ms(1);
    tmp102_req_shutdown_mode();
    nrf_delay_ms(1);
    if(m_cfg_i2c_master_init_flag)
        cfg_i2c_master_uninit();
    nrf_delay_ms(1);
    
    cfg_board_gpio_set_default();
    nfc_uninit();
    nrf_delay_ms(1);
    if(m_module_parameter.boot_nfc_unlock)
    {
        if(m_module_parameter.wkup_gpio_enable==1)
        {
            nrf_gpio_cfg_sense_input(PIN_DEF_WKUP, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
            nrf_delay_ms(1);
        }
        if(m_module_parameter.magnetic_gpio_enable)
        {
            nrf_gpio_cfg_sense_input(PIN_DEF_MAGNETIC_SIGNAL, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
            nrf_delay_ms(1);
        }
        
        NRF_NFCT->TASKS_SENSE = 1;
        NRF_NFCT->INTENCLR = 
            (NFCT_INTENCLR_RXFRAMEEND_Clear << NFCT_INTENCLR_RXFRAMEEND_Pos) |
            (NFCT_INTENCLR_RXERROR_Clear    << NFCT_INTENCLR_RXERROR_Pos);
    }
    
    nrf_delay_ms(1);
}

/**
 * @brief function for enter and wakeup deepsleep
 */
static void main_deepsleep_control(void)
{
    if(m_module_parameter.boot_nfc_unlock) 
    {
        bool magnetdet = false, wkupdet = false;
        cPrintLog(CDBG_MAIN_LOG, "wait boot unlock\n");
        if(m_module_parameter.magnetic_gpio_enable)nrf_gpio_cfg_input(PIN_DEF_MAGNETIC_SIGNAL, NRF_GPIO_PIN_NOPULL);
        if(m_module_parameter.wkup_gpio_enable==1)nrf_gpio_cfg_input(PIN_DEF_WKUP, NRF_GPIO_PIN_PULLDOWN);

        for (;; )
        {
            if(m_module_parameter.magnetic_gpio_enable){magnetdet = (nrf_gpio_pin_read(PIN_DEF_MAGNETIC_SIGNAL) == 0);}  //boot wake up level detect low for magnetic
            if(m_module_parameter.wkup_gpio_enable==1){wkupdet = (nrf_gpio_pin_read(PIN_DEF_WKUP) == 1);}  //boot wake up level detect high for wkup key
            if (mnfc_tag_on  //wake up condition
                || m_cfg_sw_reset_detected  //wake up condition
                || magnetdet  //wake up condition
                || wkupdet  //wake up condition
                || m_cfg_GPIO_wake_up_detected  //wake up condition
                || m_cfg_debug_interface_wake_up_detected
                || cTBC_check_host_connected()  //exception condition
             )
            {
                if(m_module_parameter.magnetic_gpio_enable)nrf_gpio_cfg_default(PIN_DEF_MAGNETIC_SIGNAL);
                if(m_module_parameter.wkup_gpio_enable==1)nrf_gpio_cfg_default(PIN_DEF_WKUP);
                mnfc_tag_on = false;
                cPrintLog(CDBG_FCTRL_INFO, "wakeup reason swreset:%d, gpio:%d, nfc:%d %d, jtag:%d, magnet:%d, wkup:%d\n", 
                              m_cfg_sw_reset_detected, m_cfg_GPIO_wake_up_detected, m_cfg_NFC_wake_up_detected, mnfc_tag_on, m_cfg_debug_interface_wake_up_detected, magnetdet, wkupdet);
                return;
            }
            if(cTBC_is_busy())
            {
                power_manage();  //The wakeup event is generated by the cfg_twis_board_control
            }
            else
            {
                cPrintLog(CDBG_MAIN_LOG, "enter deep sleep mode\n");
                main_prepare_power_down();
                // Enter System OFF mode.
                sd_power_system_off();
                while(1);
            }
        }
    }
}

/**
 * @brief Callback function for handling iterrupt from accelerometer.
 */

void bma250_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    char display[20];
    int len;
    sprintf(display,"g-sensor shaking\n");
    len = strlen(display);
    if(!mnus_acc_report)
    {
        if(!main_wakeup_interrupt && main_schedule_state_is_idle())
        {
            main_wakeup_reason = main_wakeup_reason_acc_event;
            main_wakeup_interrupt = true;
        }
    }
    cPrintLog(CDBG_FCTRL_INFO,"%s", display);
    cTBC_put_bypass_data((uint8_t *)display,len);
    main_ACC_ISR_detected = true;
    cTBC_event_noti("ACC_ISR");
#ifdef CDEV_NUS_MODULE
    m_nus_service_parameter.accellometer_event = '1';
#endif
}

/**
 * @brief  function for setting iterrupt from accelerometer.
 */

void cfg_bma250_interrupt_init(void)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_gpiote_in_init(PIN_DEF_ACC_INT1, &in_config, bma250_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_DEF_ACC_INT1, true);
}

/**
 * @brief Callback function for handling NFC events.
 */

static void nfc_callback(void * p_context, nfc_t2t_event_t event, const uint8_t * p_data, size_t data_length)
{
    (void)p_context;

    switch (event)
    {
        case NFC_T2T_EVENT_FIELD_ON:
            cTBC_event_noti("NFC_TAG");
            mnfc_tag_on = true;
//            LEDS_ON(BSP_LED_0_MASK);
            break;

        case NFC_T2T_EVENT_FIELD_OFF:
 //           LEDS_OFF(BSP_LED_0_MASK);
            break;

        default:
            break;
    }
}

static void sigfox_id_record_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    /** @snippet [NFC text usage_1] */
    uint32_t             err_code;
    static  uint8_t en_payload[] =
                  {'I', 'D', ':', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',' '};
    static const uint8_t en_code[] = {'e', 'n'};

//  memcpy((char*)&en_payload[3],m_module_peripheral_ID.sigfox_device_ID,4);
    cfg_bin_2_hexadecimal(m_module_peripheral_ID.sigfox_device_ID,4,(char*)&en_payload[3]);
    NFC_NDEF_TEXT_RECORD_DESC_DEF(en_text_rec,
                                  UTF_8,
                                  en_code,
                                  sizeof(en_code),
                                  en_payload,
                                  sizeof(en_payload)-1);
   /** @snippet [NFC text usage_1] */

    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,
                                       &NFC_NDEF_TEXT_RECORD_DESC(en_text_rec));
    APP_ERROR_CHECK(err_code);

}


/**
 * @brief Function for creating a record in Norwegian.
 */
static void sigfox_pac_record_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    uint32_t             err_code;
    static  uint8_t pl_payload[] =
                          {'P', 'A', 'C', ':', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
    static const uint8_t pl_code[] = {'P', 'L'};

//  memcpy((char*)&no_payload[4],m_module_peripheral_ID.sigfox_pac_code,8);
    cfg_bin_2_hexadecimal(m_module_peripheral_ID.sigfox_pac_code,8,(char*)(char*)&pl_payload[4]);
    NFC_NDEF_TEXT_RECORD_DESC_DEF(no_text_rec,
                                  UTF_8,
                                  pl_code,
                                  sizeof(pl_code),
                                  pl_payload,
                                  sizeof(pl_payload)-1);

    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,
                                       &NFC_NDEF_TEXT_RECORD_DESC(no_text_rec));
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for creating a record in Norwegian.
 */
static void ble_mac_record_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    uint32_t             err_code;
    static  uint8_t no_payload[17];
    uint8_t hex_digit[13];
    int i = 0;
    int j =0;
    static const uint8_t no_code[] = {'N', 'O'};

//  memcpy((char*)no_payload,m_module_peripheral_ID.ble_MAC,6);
    cfg_bin_2_hexadecimal(m_module_peripheral_ID.ble_MAC,6,(char*)(char*)hex_digit);
    for(i=0;i<12;)
    {
        no_payload[j++]=hex_digit[i++];
        no_payload[j++]=hex_digit[i++];
        if(j<17)
            no_payload[j++]=':';
    }
    NFC_NDEF_TEXT_RECORD_DESC_DEF(no_text_rec,
                                  UTF_8,
                                  no_code,
                                  sizeof(no_code),
                                  no_payload,
                                  sizeof(no_payload));

    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,
                                       &NFC_NDEF_TEXT_RECORD_DESC(no_text_rec));
    APP_ERROR_CHECK(err_code);
}

uint32_t nus_send_id_pac_mac(uint8_t * send_buffer)
{
    uint32_t err_code;
    send_buffer[0]='S';
    memcpy(send_buffer+1,m_module_peripheral_ID.sigfox_device_ID,4);
    err_code = ble_nus_string_send(&m_nus, (uint8_t*)send_buffer, 5);

    memset(send_buffer,0xFF,20);
    send_buffer[0]='P';
    memcpy(send_buffer+1,m_module_peripheral_ID.sigfox_pac_code,8);
    err_code = ble_nus_string_send(&m_nus, (uint8_t*)send_buffer, 9);

    memset(send_buffer,0xFF,20);
    send_buffer[0]='M';
    memcpy(send_buffer+1,m_module_peripheral_ID.ble_MAC,6);
    err_code = ble_nus_string_send(&m_nus, (uint8_t*)send_buffer,7);

    return err_code;
}

#if 0
static void android_app_record_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    uint32_t             err_code;
    static const uint8_t no_code[] = {'N', 'O'};

    /* Create NFC NDEF message description, capacity - 2 records */
//    NFC_NDEF_MSG_DEF(nfc_launchapp_msg, 1);
    NFC_NDEF_TEXT_RECORD_DESC_DEF(no_text_rec,
                                  UTF_8,
                                  no_code,
                                 sizeof(no_code),
                                  m_android_package_name,
                                  sizeof(m_android_package_name));

//    p_android_rec = nfc_android_application_rec_declare(m_android_package_name,sizeof(m_android_package_name));

 
    /* Add Android App Record as second record to message */
    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,&NFC_NDEF_TEXT_RECORD_DESC(no_text_rec));


    APP_ERROR_CHECK(err_code);
}
#endif

/**
 * @brief Function for encoding the welcome message.
 */
 
NFC_NDEF_MSG_DEF(welcome_msg, MAX_REC_COUNT);
static void welcome_msg_encode(uint8_t * p_buffer, uint32_t * p_len)
{
    sigfox_id_record_add(&NFC_NDEF_MSG(welcome_msg));
    sigfox_pac_record_add(&NFC_NDEF_MSG(welcome_msg));
    ble_mac_record_add(&NFC_NDEF_MSG(welcome_msg));

    /** @snippet [NFC text usage_2] */
    uint32_t err_code = nfc_ndef_msg_encode(&NFC_NDEF_MSG(welcome_msg),
                                            p_buffer,
                                            p_len);
    APP_ERROR_CHECK(err_code);
    /** @snippet [NFC text usage_2] */
}

void nfc_init()
{
    uint32_t err_code;

    if(!mnfc_init_flag)
    {
        /* Set up NFC */
        err_code = nfc_t2t_setup(nfc_callback, NULL);
        APP_ERROR_CHECK(err_code);

        /** @snippet [NFC URI usage_1] */
        /* Provide information about available buffer size to encoding function */
        uint32_t len = sizeof(m_ndef_msg_buf);

        welcome_msg_encode(m_ndef_msg_buf, &len);

        /** @snippet [NFC URI usage_1] */

        /* Set created message as the NFC payload */
        err_code = nfc_t2t_payload_set(m_ndef_msg_buf, len);
        APP_ERROR_CHECK(err_code);

        /* Start sensing NFC field */
        err_code = nfc_t2t_emulation_start();
        mnfc_init_flag = true;
    }
}

void nfc_uninit(void)
{
    if(mnfc_init_flag)
    {
        mnfc_init_flag = false;
        nfc_t2t_emulation_stop();
    }
}

void nfc_restart(void)
{
    if(mnfc_init_flag)
    {
        uint32_t len = sizeof(m_ndef_msg_buf);
        nfc_t2t_emulation_stop();
        nfc_ndef_msg_clear(&NFC_NDEF_MSG(welcome_msg));
        welcome_msg_encode(m_ndef_msg_buf, &len);
        nfc_t2t_payload_set(m_ndef_msg_buf, len);
        nfc_t2t_emulation_start();
    }
}


static void main_schedule_timeout_handler_examples(void * p_context)
{
    main_schedule_tick++;
}

void main_examples_prepare(void)
{
    volatile uint32_t      err_code;
    
    //timer Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    //sd init
    ble_stack_init();

    //ldo mode
    sd_power_dcdc_mode_set(1);

    //parameter init
    module_parameter_init();
    get_ble_mac_address();

    err_code = app_timer_create(&m_main_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                main_schedule_timeout_handler_examples);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(APP_MAIN_SCHEDULE_MS, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

static int user_cmd_cmd;
static int user_cmd_param_size;
static uint8_t user_cmd_param_buf[32];
static int8_t user_cmd_hitrun_ble_rssi;

static void user_cmd_hitrun_input_test(void)
{
    int int_val;
    int tick_val;
    uint8_t param_bin[16];
    bool all_test_OK;
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)
    bool check_INT_SENSOR = false;
#endif

    if(user_cmd_param_size == 2)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        int_val = param_bin[0];
    }
    else
    {
        int_val = 30;  //default val
    }
    m_hitrun_test_flag = true;
    cPrintLog(CDBG_MAIN_LOG, "input test:%d %d\n", user_cmd_param_size, int_val);
            
    if(main_get_param_val(module_parameter_item_magnetic_gpio_enable))
        main_magnet_detected = false;
    else
        main_magnet_detected = true;
    if(main_get_param_val(module_parameter_item_wkup_gpio_enable)==1)
        main_wkup_push_detected = false;
    else
        main_wkup_push_detected = true;
    mnfc_tag_on = false;
    if(int_val > 128)  //disable NFC
    {
        mnfc_tag_on = true;
        int_val -= 128;
    }
    if(!m_cfg_i2c_master_init_flag)cfg_i2c_master_init();
    main_ACC_ISR_detected = false;

#ifdef PIN_DEF_BUTTON  //CDEV_BOARD_M3
    main_button_detected = false;
#endif

    all_test_OK = false;
    if(cfg_bma250_ISR_pin_test()==NRF_SUCCESS)
    {
        nrf_delay_ms(10);
        cfg_bma250_sw_reset();
        tick_val = 0;
        while(++tick_val < (int_val * 100))
        {
            nrf_delay_ms(10);
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)  //for magnetic and key (Use the same GPIO)
            if(!check_INT_SENSOR)
            {
                if(main_magnet_detected)
                {
                    main_magnet_detected = false;
                    check_INT_SENSOR = true;
                }
            }
#endif
            if(
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHEREV2)  //for magnetic and key (Use the same GPIO)
                (check_INT_SENSOR && main_magnet_detected)
#else
                main_magnet_detected
#endif
#ifdef PIN_DEF_BUTTON  //CDEV_BOARD_M3
                && main_button_detected
#endif
                && main_wkup_push_detected
                && mnfc_tag_on
                && main_ACC_ISR_detected
            )
            {
                nrf_delay_ms(10);
                all_test_OK = true;
                break;
            }
        }
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "InputTest");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_hitrun_sense_test(void)
{
    bool all_test_OK = true;
    bool test_result;
    uint8_t param_bin[32];
    char noti_str_buf[32];
    struct bma_accel_data acc_data;
    memset(param_bin, 0, sizeof(param_bin));
    if(user_cmd_param_size == 4)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
    }
    cPrintLog(CDBG_MAIN_LOG, "sense test:%d\n", user_cmd_param_size);
    if(!m_cfg_i2c_master_init_flag)cfg_i2c_master_init();

    //ACC Test
    memset(&acc_data, 0, sizeof(acc_data));
    if((cfg_bma250_sw_reset() == NRF_SUCCESS) && cfg_bma250_read_xyz(&acc_data) == true)
    {
        sprintf(noti_str_buf,"ACCTest%02x%02x%02x", (uint8_t)acc_data.x, (uint8_t)acc_data.y, (uint8_t)acc_data.z);
        cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
    }
    else
    {
        cTBC_usrcmd_msg_noti(user_cmd_cmd, false, "ACCErr");
        all_test_OK = false;
    }

    //ADC Test
#ifdef PIN_DEF_BATTERY_ADC_INPUT
    {
        adc_configure();
        cfg_bas_timer_start();
        nrf_delay_ms(1000);  //wait adc result check plz BATTERY_LEVEL_AVG_CNT and BATTERY_LEVEL_MEAS_INTERVAL
        sprintf(noti_str_buf,"ADCTest%02x", avg_report_volts);
        if(((param_bin[0] != 0) && (param_bin[1] != 0))
            && ((avg_report_volts < param_bin[0]) || (param_bin[1] < avg_report_volts))
        )
        {
            all_test_OK = false;
            test_result = false;
        }
        else
        {
            test_result = true;
        }
        cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
    }
#endif

    //tmp sensor Test
    {
        extern uint32_t tmp102_set_config(void);
        extern uint32_t tmp102_get_tmp_data_once(int16_t *tmp102a, int16_t *tmp102b);
        int16_t tmp_a, tmp_b;
        if((tmp102_set_config()==NRF_SUCCESS)
            && (tmp102_get_tmp_data_once(&tmp_a, &tmp_b)==NRF_SUCCESS)
        )
        {
            test_result = true;
            sprintf(noti_str_buf,"TMPTest%02x%02x", (uint8_t)tmp_a, (uint8_t)tmp_b);
        }
        else
        {
            all_test_OK = false;
            test_result = false;
            sprintf(noti_str_buf,"TMPTest");
        }
        cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "SenseTest");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_hitrun_ble_scan_handler(ble_gap_evt_adv_report_t * p_adv_report)
{
    uint8_t *p_d;

    p_d = p_adv_report->data;
    if(p_d[5]=='S' && p_d[6]=='F' && p_d[7]=='M' && p_d[8]=='T' && p_d[9]=='E' && p_d[10]=='S' && p_d[11]=='T')  //SFMTEST0000
    {
        if(user_cmd_hitrun_ble_rssi == 0 || user_cmd_hitrun_ble_rssi < p_adv_report->rssi)
        {
            user_cmd_hitrun_ble_rssi = p_adv_report->rssi;
            cPrintLog(CDBG_MAIN_LOG, "ble test:%d\n", user_cmd_hitrun_ble_rssi);
        }
    }
#if 0
    cPrintLog(CDBG_MAIN_LOG,"Mac:%02x%02x%02x%02x%02x%02x\r\n",
             p_adv_report->peer_addr.addr[0],
             p_adv_report->peer_addr.addr[1],
             p_adv_report->peer_addr.addr[2],
             p_adv_report->peer_addr.addr[3],
             p_adv_report->peer_addr.addr[4],
             p_adv_report->peer_addr.addr[5]
             );
    cPrintLog(CDBG_MAIN_LOG,"Data %02x%02x%02x%02x%02x%02x%02x\r\n",p_d[5],p_d[6],p_d[7],p_d[8],p_d[9],p_d[10],p_d[11]);
#endif
}

static void user_cmd_hitrun_led_test(void)
{
    bool all_test_OK = true;
    bool test_result;
    int8_t wifi_rssi = 0, ble_rssi = 0;
    uint8_t gps_time = 0;
    uint8_t param_bin[32];
    char noti_str_buf[32];
    int i;

    advertising_start(false, false);
    memset(param_bin, 0, sizeof(param_bin));
    if(user_cmd_param_size == 6)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        wifi_rssi = (int8_t)param_bin[0];
        ble_rssi = (int8_t)param_bin[1];
        gps_time = param_bin[2];
        if(wifi_rssi || ble_rssi || gps_time)
        {
            cPrintLog(CDBG_MAIN_LOG, "Rf test enable WIFI:%d BLE:%d GPS:%d\n", wifi_rssi, ble_rssi, gps_time);
        }
    }
    cPrintLog(CDBG_MAIN_LOG, "Led test:%d\n", user_cmd_param_size);

#ifdef CDEV_GPS_MODULE
    cGps_power_control(true, true);
#endif

    //led test 1
    cfg_ble_led_control(true);
    nrf_delay_ms(1000);

    //led test 2
#ifdef CDEV_WIFI_MODULE
    //disable wifi log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_ERR));

    cWifi_set_retry_time(1);
    
    if(cWifi_ap_get_available_first_BSSID("SFMTEST") != CWIFI_Result_OK)
    {
        cPrintLog(CDBG_MAIN_LOG, "WIFI scan Error!\n");
        all_test_OK = false;
    }
    if(all_test_OK)
    {       
        uint32_t wifi_get_cnt;
        int32_t *rssi;
        cfg_ble_led_control(false);
        if(cfg_is_3colorled_contorl())
        {
            nrf_delay_ms(500);
            cfg_wkup_output_control(true);
        }


        
        while(!(!cWifi_is_scan_state() && !cWifi_bus_busy_check()));  //wait scan
        nrf_delay_ms(10);
        if(cfg_is_3colorled_contorl())cfg_wkup_output_control(false);
        
        if(wifi_rssi)
        {
            if((cWifi_get_scan_result(&wifi_get_cnt, NULL, &rssi, NULL) == CWIFI_Result_OK)
                && (wifi_get_cnt > 0)
                && (rssi[0] > wifi_rssi)
            )
            {
                test_result = true;
                sprintf(noti_str_buf,"WIFITest%d", (int)rssi[0]);

            }
            else
            {
                test_result = false;
                all_test_OK = false;
                sprintf(noti_str_buf,"WIFITest");
                cPrintLog(CDBG_MAIN_LOG, "WIFI rssi Error!\n");
            }
            cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
        }
    }
#else
    cfg_ble_led_control(false);
    if(cfg_is_3colorled_contorl())cfg_wkup_output_control(true);
    nrf_delay_ms(1000);
    if(cfg_is_3colorled_contorl())cfg_wkup_output_control(false);
#endif

    //led test 3
    if(all_test_OK)
    {
        //disable sigfox log
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_SIGFOX_INFO));
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_SIGFOX_ERR));
        if((sigfox_bypass_req(NULL, NULL) == NRF_SUCCESS)
#ifdef CDEV_WIFI_MODULE
            && (cWifi_bypass_req(NULL, NULL) == CWIFI_Result_OK)
#endif
        )
        {
            nrf_delay_ms(1000);
            sigfox_bypass_write_request("AT$CB=-1,1\r\n", 12);
            nrf_delay_ms(1000);
            sigfox_bypass_write_request("AT$CB=-1,0\r\n", 12);
        }
        else
        {
            all_test_OK = false;
        }
    }

    //led test all
    if(all_test_OK)
    {
        cfg_ble_led_control(true);
#ifdef CDEV_WIFI_MODULE
        cWifiState_bypass_write_request("AT+LEDON\r\n", 10);
        nrf_delay_ms(500);
        cWifi_bus_enable(false);
#endif
        sigfox_bypass_write_request("AT$CB=-1,1\r\n", 12);
        if(cfg_is_3colorled_contorl())cfg_wkup_output_control(true);
    }

    if(all_test_OK && ble_rssi)
    {
        user_cmd_hitrun_ble_rssi = 0;
        ble_scan_start(user_cmd_hitrun_ble_scan_handler);
        nrf_delay_ms(1000);
        for(i=0; i < 20; i++)
        {
            if(user_cmd_hitrun_ble_rssi)break;
            nrf_delay_ms(100);
        }
        ble_scan_stop();

        if(user_cmd_hitrun_ble_rssi && (user_cmd_hitrun_ble_rssi >= ble_rssi))
        {
            test_result = true;
        }
        else
        {
            cPrintLog(CDBG_MAIN_LOG, "BLE scan Error! %d, %d\n", ble_rssi, user_cmd_hitrun_ble_rssi);
            test_result = false;
            all_test_OK = false;
        }
        sprintf(noti_str_buf,"BLETest%d", user_cmd_hitrun_ble_rssi);
        cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
            
    }

    if(all_test_OK && gps_time)
    {
#ifdef CDEV_GPS_MODULE
        extern void set_CN0_check_type(int val);
        uint32_t    gps_acquire_tracking_time_sec_old;
        gps_acquire_tracking_time_sec_old = m_module_parameter.gps_acquire_tracking_time_sec;
        m_module_parameter.gps_acquire_tracking_time_sec = gps_time;

        //disable gps log
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_GPS_INFO));
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_GPS_ERR));

        set_CN0_check_type(0);
        while(cGps_status_available()!=CGPS_Result_OK)nrf_delay_ms(200);
        cGps_nmea_acquire_request();
        nrf_delay_ms(200);

        while(1)
        {
            if(cGps_acquire_tracking_check()==CGPS_Result_OK)break;
            if(!cGps_bus_busy_check())break;
            nrf_delay_ms(100);
        }
        while(cGps_bus_busy_check());
        m_module_parameter.gps_acquire_tracking_time_sec = gps_acquire_tracking_time_sec_old;
        cGps_power_control(true, false);
        test_result = (cGps_acquire_tracking_check()==CGPS_Result_OK);
        if(!test_result)
        {
            cPrintLog(CDBG_MAIN_LOG, "GPS Error! %d, %d\n", cGps_acquire_tracking_check(), cGps_bus_busy_check());
            all_test_OK = false;
        }
        sprintf(noti_str_buf,"GPSTest");
        cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
#endif
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "LedTest");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_set_sigfox_power(void)
{
    uint8_t param_bin[16];
    uint8_t sigfox_tx_pwr;
    bool all_test_OK = false;

    if(user_cmd_param_size == 2)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        sigfox_tx_pwr = param_bin[0];
        cPrintLog(CDBG_MAIN_LOG, "Sigfox Tx Pwr:%d\n", sigfox_tx_pwr);
        if(cfg_sigfox_set_powerlevel(sigfox_tx_pwr))
        {
            all_test_OK = true;
        }
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "SigfoxTxPwr");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_set_wifi_power(void)
{
    bool all_test_OK = false;
    uint8_t param_bin[16];

    if(user_cmd_param_size == 12)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        cPrintLog(CDBG_MAIN_LOG, "WIFI Tx Tables:%d,%d,%d,%d,%d,%d\n", param_bin[0], param_bin[1], param_bin[2], param_bin[3], param_bin[4], param_bin[5]);
#ifdef CDEV_WIFI_MODULE
        if(cWifi_bypass_req(NULL, NULL) == CWIFI_Result_OK)
        {
            char sendAtCmd[32];
            int sendATCmdSize;
            int timeout;

            timeout = 5000;
            while(!cWifiState_is_bypass_mode())
            {
                if(--timeout==0)break;  //wait bypassmode
                nrf_delay_ms(1);
            }

            if(timeout > 0)
            {
/************/
//AT Cmd Examples (Tx Power Table)
/* AT+TXPREAD           // Default 524E4A444038                      */
/* RNJD@8                                                            */
/* OK                                                                */
/* AT+TXPWRITE="445544553322"   // 445544553322                      */
/* WRITTEN                                                           */
/* OK                                                                */
/* AT+TXPREAD                                                        */
/* DUDU3"                                                            */
/* OK                                                                */
/* AT+TXPWRITE="524E4A444038"  // Default 524E4A444038 rewrite       */
/* WRITTEN                                                           */
/* OK                                                                */
/* AT+TXPREAD                                                        */
/* RNJD@8                                                            */
/* OK                                                                */
/*************/
                sendATCmdSize = sprintf((char *)sendAtCmd, "AT+TXPWRITE=\"%02X%02X%02X%02X%02X%02X\"\r\n", param_bin[0], param_bin[1], param_bin[2], param_bin[3], param_bin[4], param_bin[5]);
                cPrintLog(CDBG_MAIN_LOG, "send to WIFI :%s", sendAtCmd);
                cWifiState_bypass_write_request(sendAtCmd, sendATCmdSize);
                all_test_OK = true;
                nrf_delay_ms(500);
            }
        }
#endif
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "WIFITxPwr");
    m_cTBC_dbg_mode_run_func = NULL;
}

void dbg_i2c_user_cmd_proc(int cmd, int param_size, const uint8_t *param)
{
    cPrintLog(CDBG_MAIN_LOG, "user cmd : 0x%02x, param size:%d\n", cmd, param_size);
    switch(cmd /*CTBC_CMD_TYPE*/)
    {
        case 0x80:  //led on //CTBC_CMD_USER_START
            cPrintLog(CDBG_MAIN_LOG, "led on\n");
            if(cfg_is_3colorled_contorl())
            {
                cfg_ble_led_control(true);
                cfg_wkup_output_control(true);
            }
            else
            {
                cfg_ble_led_control(true);
            }
            break;

        case 0x81:  //led off
            cPrintLog(CDBG_MAIN_LOG, "led off\n");
            if(cfg_is_3colorled_contorl())
            {
                cfg_ble_led_control(false);
                cfg_wkup_output_control(false);
            }
            else
            {
                cfg_ble_led_control(false);
            }
            break;

        case 0x82:  //input test
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_hitrun_input_test;
            }
            break;

        case 0x83:  //sensor_test
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_hitrun_sense_test;
            }
            break;

        case 0x84:  //led_test (and rf test)
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_hitrun_led_test;
            }
            break;

        case 0x85:  //Set Sigfox Tx Power
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_set_sigfox_power;
            }
            break;

        case 0x86:  //Set Wifi Tx Power in flash
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_set_wifi_power;
            }
            break;

        default:
            break;
    }
}

static void tbc_over_rtt_sec_tick_proc(void)
{
    main_Sec_tick++;
#ifdef PIN_DEF_GPS_BKUP_CTRL_WITH_PULLUP  //GPS_BKUP_CTRL
    if(main_GPS_BKUP_time_out > 0)  //GPS_BKUP_CTRL
    {
        if(--main_GPS_BKUP_time_out == 0)
        {
            cfg_board_GPS_BKUP_ctrl(false);
            cPrintLog(CDBG_FCTRL_INFO, "GPS_BKUP Off\n");
        }
    }
#endif

}

#if 0  //sleep test code
APP_TIMER_DEF(m_test_led_blink_timer_id);
void main_test_for_sleep_timer_handler(void * p_context)
{
    static int timer_tick = 0;
    (void)p_context;

    if(++timer_tick % 10 == 0)
    {
        cfg_ble_led_control(true);
    }
    else
    {
        cfg_ble_led_control(false);
    }
}

void main_test_for_sleep(void)
{
    cPrintLog(CDBG_MAIN_LOG, "Sleep Test Mode\n");
    nrf_delay_ms(2000);
    cPrintLog(CDBG_MAIN_LOG, "Prepare Sleep\n");

    //sensor power down
    if(!m_cfg_i2c_master_init_flag)
        cfg_i2c_master_init();
    nrf_delay_ms(1);
    bma250_req_suppend_mode();
    nrf_delay_ms(1);
    tmp102_req_shutdown_mode();
    nrf_delay_ms(1);
    if(m_cfg_i2c_master_init_flag)
        cfg_i2c_master_uninit();
    nrf_delay_ms(1);

    //prepare led blink
    app_timer_create(&m_test_led_blink_timer_id, APP_TIMER_MODE_REPEATED, main_test_for_sleep_timer_handler);
    
    advertising_start(false, true);  //stop ble advertising
    nfc_uninit();  //stop nfc

#ifdef CDEV_GPS_MODULE  //stop gps backup battery
    cGps_power_control(false, false);  //gps power off
    nrf_delay_ms(1);

    //gps backup battery unuse
    nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
    nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 1);
    nrf_delay_ms(10);
    nrf_gpio_pin_write(PIN_DEF_GPS_RESET, 0);
    nrf_delay_ms(1);
    nrf_gpio_cfg_output(PIN_DEF_GPS_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 1);
    nrf_delay_ms(1);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 0);
    nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);

    cfg_board_gpio_set_default_gps(); //gpio relese for gps
#endif

    cfg_board_gpio_set_default();  //release gpio
    nrf_gpio_cfg_output(PIN_DEF_BLE_LED_EN);  //led gpio settup
    app_timer_start(m_test_led_blink_timer_id, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
    
    cPrintLog(CDBG_MAIN_LOG, "Start Sleep\n");
    while(1)power_manage();
    
}
#endif

#ifdef TEST_FEATURE_SIGFOX_CW
static void cfg_enter_sigfox_CW_mode(void)
{
    cPrintLog(CDBG_MAIN_LOG, "prepare sigfox CW mode\n");
    cfg_board_gpio_set_default_gps();
    if(!m_cfg_i2c_master_init_flag)
        cfg_i2c_master_init();
    nrf_delay_ms(1);
    bma250_req_suppend_mode();
    nrf_delay_ms(1);
    tmp102_req_shutdown_mode();
    nrf_delay_ms(1);
    if(m_cfg_i2c_master_init_flag)
        cfg_i2c_master_uninit();
    nrf_delay_ms(1);
    cfg_sigfox_prepare_start();
    nrf_delay_ms(1000);

    if(sigfox_bypass_req(NULL, NULL) == NRF_SUCCESS)
    {
        nrf_delay_ms(3000);
        cPrintLog(CDBG_MAIN_LOG, "start sigfox CW:%s", SIGFOX_CW_CMD);
        sigfox_bypass_write_request(SIGFOX_CW_CMD, strlen(SIGFOX_CW_CMD));
    }
    else
    {
        cPrintLog(CDBG_MAIN_LOG, "start sigfox CW ERROR!");
    }
    while(1)power_manage();
}
#endif

int main(void)
{
    bool    erase_bonds = 0;

    cPrintLog(CDBG_MAIN_LOG, "\n====== %s Module Started Ver:%s bdtype:%d======\n", m_cfg_model_name, m_cfg_sw_ver, m_cfg_board_type);
    cPrintLog(CDBG_MAIN_LOG, "build date:%s, %s\n", m_cfg_build_date, m_cfg_build_time);
    module_parameter_early_read();

    cfg_examples_check_enter();
    cfg_board_early_init();
    main_wakeup_interrupt = false;

    //state Initialize
    m_module_mode = ACC;
    m_init_excute = true;
    sigfox_set_state(SETUP_S);
    bma250_set_state(NONE_ACC);
    tmp102_set_state(NONE_TMP);

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    ble_stack_init();
    sd_power_dcdc_mode_set(1);
    module_parameter_init();

#ifdef TEST_FEATURE_SIGFOX_CW  //for cw test
    cfg_enter_sigfox_CW_mode();
#endif

#ifdef CDEV_NUS_MODULE
    nus_data_init();
#endif


#if (CDEV_BOARD_TYPE == CDEV_BOARD_M3)  // M3 : I2c0_SCL_DBG->BUTTON,  I2c0_SDA_DBG->BCKP_GPS  
    cTBC_init(dbg_i2c_user_cmd_proc, false);  // not use I2C Slave
#else
    cTBC_init(dbg_i2c_user_cmd_proc, true);
#endif
    cTBC_OVER_RTT_init(tbc_over_rtt_sec_tick_proc); //depend on cTBC_init() //FEATURE_CFG_RTT_MODULE_CONTROL

    get_ble_mac_address();

#if (CDEV_BOARD_TYPE == CDEV_BOARD_M3)  // M3 : I2c0_SCL_DBG->BUTTON,  I2c0_SDA_DBG->BCKP_GPS  
    cfg_ble_led_control(true);
    nrf_delay_ms(200);
    cfg_ble_led_control(false);
#endif

    nfc_init();
    main_deepsleep_control();
    cfg_board_gpio_set_default_gps();

    cfg_board_init();
    if(module_parameter_ID_value_update())
    {
        cPrintLog(CDBG_FCTRL_INFO, "nfc reinit\n");
        nfc_restart();
    }

    if(main_get_param_val(module_parameter_item_magnetic_gpio_enable))cfg_magnetic_sensor_init(main_magnet_attach_callback, NULL);
    if(main_get_param_val(module_parameter_item_wkup_gpio_enable)==1)cfg_wkup_gpio_init(main_wkup_det_callback);
#ifdef PIN_DEF_BUTTON  //CDEV_BOARD_M3
    cfg_button_init(1 /*Active High*/, PIN_DEF_BUTTON, main_button_short_press_callback, main_button_long_press_callback);
#endif

//    nfc_init();
//    sigfox_power_on(true);
//    nrf_delay_ms(1000);
    cPrintLog(CDBG_FCTRL_INFO, "%s started!\n", __func__);
//    cfg_bma250_spi_init();

    cfg_i2c_master_init();
    cfg_bma250_interrupt_init();
    bma250_read_chip();
//    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
//    APP_ERROR_CHECK(err_code);
    peer_manager_init(erase_bonds);

    gap_params_init();
    services_init();


    advertising_init();
//    services_init();
    conn_params_init();

    main_timer_create();
//    cfg_sigfox_timer_create();
    cfg_bma250_timer_create();
    cfg_tmp102_timer_create();
#ifdef CDEV_NUS_MODULE
//    nus_timer_create();
#endif

#ifdef PIN_DEF_BATTERY_ADC_INPUT
    cfg_bas_timer_create();
#endif

    // Start execution.
    cPrintLog(CDBG_FCTRL_INFO, "%s main schedule start! state:%d\n", __func__, m_module_mode);
    if(m_module_parameter.scenario_mode == MODULE_SCENARIO_ASSET_TRACKER)
    {
        if(m_module_parameter.start_wait_time_for_ctrl_mode_change_sec)  //use to sigfox dl enable
            cfg_sigfox_downlink_on_off(true);
        else
            cfg_sigfox_downlink_on_off(false);
#if 0 //(CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE_MINI) //disable nfc tag restart
        mnfc_tag_on_CB = main_timer_schedule_restart_check_idle;
#endif
    }
    else if(m_module_parameter.scenario_mode == MODULE_SCENARIO_MWC_DEMO)
    {            
        //set start state
        m_module_mode = MAIN_SCENARIO_LOOP;
        if(m_module_parameter.start_wait_time_for_ctrl_mode_change_sec)  //use to sigfox dl enable
            cfg_sigfox_downlink_on_off(true);
        else
            cfg_sigfox_downlink_on_off(false);
    }
    else if(m_module_parameter.scenario_mode == MODULE_SCENARIO_IHERE_MINI)
    {
        if(m_module_parameter.start_wait_time_for_ctrl_mode_change_sec)  //use to sigfox dl enable
            cfg_sigfox_downlink_on_off(true);
        else
            cfg_sigfox_downlink_on_off(false);
        mnfc_tag_on_CB = ihere_mini_current_schedule_start;
    }
    advertising_start(true, true);

    cPrintLog(CDBG_FCTRL_INFO, "BLE MAC:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.ble_MAC, 6);
    cPrintLog(CDBG_FCTRL_INFO, "SFX ID:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.sigfox_device_ID, 4);
    cPrintLog(CDBG_FCTRL_INFO, "SFX PAC CODE:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.sigfox_pac_code, 8);   
    cPrintLog(CDBG_FCTRL_INFO, "WIFI MAC:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.wifi_MAC_STA, 6);    

#ifdef CDEV_WIFI_MODULE
    {
        uint8_t wifi_app_ver;
        uint16_t initDataVer;
        
        cWifi_get_version_info(&wifi_app_ver, &initDataVer);
        cPrintLog(CDBG_FCTRL_INFO, "WIFI AppVer:%02x, InitDataVer:%04x\n", wifi_app_ver, initDataVer);
    }
#endif

    cTBC_check_N_enter_bypassmode(200, main_bypass_enter_CB, main_bypass_exit_CB);
    if(m_hitrun_test_flag)NVIC_SystemReset();
    // Enter main loop.
//    main_test_for_sleep();  //sleep test
    main_wakeup_reason = main_wakeup_reason_powerup;
    main_timer_schedule_start();

    for (;; )
    {
        if(main_wakeup_interrupt == true)
        {
            main_wakeup_interrupt = false;
            main_timer_schedule_restart_check_idle();
        }

        if(mnfc_tag_on)
        {
            mnfc_tag_on= false;
//          advertising_start(false, led_control);
//          advertising_start(true, led_control);            
            if(mnfc_tag_on_CB)mnfc_tag_on_CB();
        }

        if(main_powerdown_request)
        {
            int i;
            cPrintLog(CDBG_MAIN_LOG, "proc powerdown\n");
            main_powerdown_request = false;
            app_timer_stop_all();
            for(i=0;i<20;i++)
            {
                cfg_ble_led_control(((i%2==0)?true:false));
                nrf_delay_ms(100);
            }
            cfg_board_gpio_set_default_gps();
            main_prepare_power_down();
            cPrintLog(CDBG_MAIN_LOG, "power off\n");
#ifdef CDEV_GPS_MODULE  //stop gps backup battery
#ifdef PIN_DEF_GPS_BKUP_CTRL_WITH_PULLUP
            cfg_board_gpio_set_default_gps(); //gpio relese for gps
            cfg_board_GPS_BKUP_ctrl(false);  //GPS_BKUP_CTRL
#else
            //gps backup battery unuse
            nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
            nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 1);
            nrf_delay_ms(10);
            nrf_gpio_pin_write(PIN_DEF_GPS_RESET, 0);
            nrf_delay_ms(1);
            nrf_gpio_cfg_output(PIN_DEF_GPS_PWR_EN);
            nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 1);
            nrf_delay_ms(1);
            nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 0);
            nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);
            nrf_gpio_cfg_default(PIN_DEF_2ND_POW_EN);
            cfg_board_gpio_set_default_gps(); //gpio relese for gps
#endif
#endif
            sigfox_power_on(false);
            sd_power_system_off();
            while(1);
        }

        if(m_module_parameter_save_N_reset_req)
        {
            m_module_parameter_save_N_reset_req = false;
            m_module_parameter_update_req = true;
            module_parameter_check_update();
            nrf_delay_ms(1000);
            NVIC_SystemReset(); 
        }
        power_manage();
    }
}


/**
 * @}
 */
