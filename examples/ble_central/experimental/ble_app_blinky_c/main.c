/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define CENTRAL_LINK_COUNT        1                                        /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT     0                                        /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE      GATT_MTU_SIZE_DEFAULT                    /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define UART_TX_BUF_SIZE          256                                      /**< Size of the UART TX buffer, in bytes. Must be a power of two. */
#define UART_RX_BUF_SIZE          1                                        /**< Size of the UART RX buffer, in bytes. Must be a power of two. */

#define CENTRAL_SCANNING_LED      BSP_LED_0_MASK                           /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED     BSP_LED_1_MASK                           /**< Connected LED will be on when the device is connected. */

#define APP_TIMER_PRESCALER       0                                        /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS      (2 + BSP_APP_TIMERS_NUMBER)                /**< Maximum number of timers used by the application. */
#define APP_TIMER_OP_QUEUE_SIZE   2                                        /**< Size of timer operation queues. */

#define SEC_PARAM_BOND            1                                        /**< Perform bonding. */
#define SEC_PARAM_MITM            0                                        /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE                     /**< No I/O capabilities. */
#define SEC_PARAM_OOB             0                                        /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                                        /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE    16                                       /**< Maximum encryption key size in octets. */

#define SCAN_INTERVAL             0x00A0                                   /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                                   /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT              0x0000                                   /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)         /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)          /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                        /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)          /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE               2                                        /**< Size of a UUID, in bytes. */

#define LEDBUTTON_LED             BSP_LED_2_MASK                           /**< LED to indicate a change of state of the the Button characteristic on the peer. */
#define LEDBUTTON_BUTTON_PIN      BSP_BUTTON_0                             /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50, APP_TIMER_PRESCALER) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

static const char m_target_periph_name[] = "Nordic_Blinky";                /**< Name of the device we try to connect to. This name is searched in the scan report data*/


/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    0,
    (uint16_t)SUPERVISION_TIMEOUT
};

static ble_lbs_c_t        m_ble_lbs_c;        /**< Main structure used by the LBS client module. */
static ble_db_discovery_t m_ble_db_discovery; /**< DB structure used by the database discovery module. */


/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    LEDS_CONFIGURE(CENTRAL_SCANNING_LED | CENTRAL_CONNECTED_LED | LEDBUTTON_LED);
    LEDS_OFF(CENTRAL_SCANNING_LED | CENTRAL_CONNECTED_LED | LEDBUTTON_LED);
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_scan_stop();
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);

    LEDS_OFF(CENTRAL_CONNECTED_LED);
    LEDS_ON(CENTRAL_SCANNING_LED);
}


/**@brief Handles events coming from the LED Button central module.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
                                                p_lbs_c_evt->conn_handle,
                                                &p_lbs_c_evt->params.peer_db);
            NRF_LOG_INFO("LED Button service discovered on conn_handle 0x%x\r\n", p_lbs_c_evt->conn_handle);

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
            NRF_LOG_INFO("Button state changed on peer to 0x%x\r\n", p_lbs_c_evt->params.button.button_state);
            if (p_lbs_c_evt->params.button.button_state)
            {
                LEDS_ON(LEDBUTTON_LED);
            }
            else
            {
                LEDS_OFF(LEDBUTTON_LED);
            }
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    uint32_t err_code;
    data_t   adv_data;
    bool     do_connect = false;
    data_t   dev_name;

    // For readibility.
    const ble_gap_evt_t * const  p_gap_evt  = &p_ble_evt->evt.gap_evt;
    const ble_gap_addr_t * const peer_addr  = &p_gap_evt->params.adv_report.peer_addr;
    {
        // Initialize advertisement report for parsing
        adv_data.p_data     = (uint8_t *)p_gap_evt->params.adv_report.data;
        adv_data.data_len   = p_gap_evt->params.adv_report.dlen;


        //search for advertising names
        bool found_name = false;
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                    &adv_data,
                                    &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // Look for the short local name if it was not found as complete
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                        &adv_data,
                                        &dev_name);
            if (err_code != NRF_SUCCESS)
            {
                // If we can't parse the data, then exit
                return;
            }
            else
            {
                found_name = true;
            }
        }
        else
        {
            found_name = true;
        }
        if (found_name)
        {
            if (strlen(m_target_periph_name) != 0)
            {
                if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len )== 0)
                {
                    do_connect = true;
                }
            }
        }
    }

    if (do_connect)
    {
        // Initiate connection.
        err_code = sd_ble_gap_connect(peer_addr, &m_scan_params, &m_connection_param);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 * parses scanning reports, initiating a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report central applications activity.
 *
 * @note Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *       should be dispatched to the target application before invoking this function.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static void on_ble_evt(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;

    // For readability.
    const ble_gap_evt_t * const p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.\r\n");
            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_ble_db_discovery, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            LEDS_ON(CENTRAL_CONNECTED_LED);
            LEDS_OFF(CENTRAL_SCANNING_LED);
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.\r\n");
            scan_start();
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("[APP]: Connection Request timed out.\r\n");
            }
        } break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 * been received.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_lbs_c_on_ble_evt(&m_ble_lbs_c, p_ble_evt);

}


/**@brief LED Button client initialization.
 */
static void lbs_c_init(void)
{
    uint32_t         err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
            err_code = ble_lbs_led_status_send(&m_ble_lbs_c, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("LBS write LED state %d\r\n", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);
}


/**
 * @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}



/** @brief Function to sleep until a BLE event is received by the application.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    leds_init();
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    buttons_init();
    ble_stack_init();

    db_discovery_init();
    lbs_c_init();

    // Start scanning for peripherals and initiate connection to devices which
    // advertise.
    scan_start();

    NRF_LOG_INFO("\r\nBlinky Start!\r\n");

    // Turn on the LED to signal scanning.
    LEDS_ON(CENTRAL_SCANNING_LED);

    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            // Wait for BLE events.
            power_manage();
        }
    }
}
