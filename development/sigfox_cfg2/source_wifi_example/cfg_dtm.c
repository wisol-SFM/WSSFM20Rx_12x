/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
  @defgroup dtm_standalone main.c
  @{
  @ingroup ble_sdk_app_dtm_serial
  @brief Stand-alone DTM application for UART interface.

 */

#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "fstorage.h"
#include "ble_dtm.h"
#include "boards.h"
#include "cfg_board_def.h"
#include "ble.h"
#include "softdevice_handler.h"
#include "nrf_dfu_settings.h"
#include "nrf_sdm.h"

//#define TEST_FEATURE_UART_ECHO
#define FEATURE_DFU_SUPPORT  //download mode key goto bootloader
#define FEATURE_TEST_LED_ON_OFF  // 'a' -> led on, 'b' -> Led off

#ifdef FEATURE_DFU_SUPPORT
#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif
#endif

extern void nrf_dfu_settings_init(void);


// @note: The BLE DTM 2-wire UART standard specifies 8 data bits, 1 stop bit, no flow control.
//        These parameters are not configurable in the BLE standard.

/**@details Maximum iterations needed in the main loop between stop bit 1st byte and start bit 2nd
 * byte. DTM standard allows 5000us delay between stop bit 1st byte and start bit 2nd byte.
 * As the time is only known when a byte is received, then the time between between stop bit 1st
 * byte and stop bit 2nd byte becomes:
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration):
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations.
 *
 * This is rounded down to 21.
 *
 * @note If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE ((5000 + 2 * UART_POLL_CYCLE) / UART_POLL_CYCLE)

/**@brief Function for UART initialization.
 */
static void uart_init(void)
{
    // Configure UART0 pins.
    nrf_gpio_cfg_output(PIN_DEF_DTM_TX);
    nrf_gpio_cfg_input(PIN_DEF_DTM_RX, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELTXD       = PIN_DEF_DTM_TX;
    NRF_UART0->PSELRXD       = PIN_DEF_DTM_RX;
    NRF_UART0->BAUDRATE      = DTM_BITRATE;

    // Clean out possible events from earlier operations
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->EVENTS_ERROR  = 0;

    // Activate UART.
    NRF_UART0->ENABLE        = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->INTENSET      = 0;
    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
}


/**@brief Function for splitting UART command bit fields into separate command parameters for the DTM library.
*
 * @param[in]   command   The packed UART command.
 * @return      result status from dtmlib.
 */
static uint32_t dtm_cmd_put(uint16_t command)
{
    dtm_cmd_t      command_code = (command >> 14) & 0x03;
    dtm_freq_t     freq         = (command >> 8) & 0x3F;
    uint32_t       length       = (command >> 2) & 0x3F;
    dtm_pkt_type_t payload      = command & 0x03;

    // Check for Vendor Specific payload.
    if (payload == 0x03)
    {
        /* Note that in a HCI adaption layer, as well as in the DTM PDU format,
           the value 0x03 is a distinct bit pattern (PRBS15). Even though BLE does not
           support PRBS15, this implementation re-maps 0x03 to DTM_PKT_VENDORSPECIFIC,
           to avoid the risk of confusion, should the code be extended to greater coverage.
        */
        payload = DTM_PKT_VENDORSPECIFIC;
    }
    return dtm_cmd(command_code, freq, length, payload);
}

#if 0
void flash_callback(fs_evt_t const * const evt, fs_ret_t result)
{
    if (result == FS_SUCCESS)
    {
        nrf_gpio_pin_clear(PIN_DEF_BLE_LED_EN);
        nrf_delay_ms(200);
        NVIC_SystemReset();
    }
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);
}

static nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
static ble_enable_params_t ble_enable_params;

void bootloader_check_enter(void)
{
    uint32_t err_code;
    volatile int dl_pin = 0;
    int i;

    nrf_gpio_cfg_input(PIN_DEF_WIFI_INT, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(100);

#if 0
    while(1)
    {
        dl_pin = nrf_gpio_pin_read(PIN_DEF_WIFI_INT);
        nrf_delay_ms(500);
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, dl_pin);
    };
#endif

    dl_pin = nrf_gpio_pin_read(PIN_DEF_WIFI_INT);
    if(!dl_pin)  //wifo download mode enable check
    {
        for(i=0; i<10; i++)
        {
            if(i%2 == 0)
            {
                nrf_gpio_pin_set(PIN_DEF_BLE_LED_EN);
            }
            else
            {
                nrf_gpio_pin_clear(PIN_DEF_BLE_LED_EN);
            }
            nrf_delay_ms(300);
        }
        // Initialize the SoftDevice handler module.
        SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
        err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                        PERIPHERAL_LINK_COUNT,
                                                        &ble_enable_params);
        APP_ERROR_CHECK(err_code);
        
        //Check the ram settings against the used number of links
        CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
#if (NRF_SD_BLE_API_VERSION == 3)
        ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
        
        // Enable BLE stack.
        err_code = softdevice_enable(&ble_enable_params);
        APP_ERROR_CHECK(err_code);

        err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
        APP_ERROR_CHECK(err_code);
        
        err_code = nrf_dfu_flash_init(true);
        APP_ERROR_CHECK(err_code);

        nrf_dfu_settings_init();

        nrf_delay_ms(1);
        if(!s_dfu_settings.enter_buttonless_dfu)
        {
            s_dfu_settings.enter_buttonless_dfu = true;
            (void)nrf_dfu_settings_write(flash_callback);
            while(1);
        }
        else
        {
            nrf_gpio_pin_clear(PIN_DEF_BLE_LED_EN);
            nrf_delay_ms(200);
            NVIC_SystemReset();
        }
    }
}
#endif

#ifdef TEST_FEATURE_UART_ECHO
static void uart_echo_test(void)
{
    uint8_t     rx_byte;                   // Last byte read from UART.
    volatile int out_lvl = 0;
    while(1)
    {
        if (NRF_UART0->EVENTS_RXDRDY == 0)
        {
            // Nothing read from the UART.
            continue;
        }
        NRF_UART0->EVENTS_RXDRDY = 0;
        rx_byte                  = (uint8_t)NRF_UART0->RXD;
        NRF_UART0->TXD = rx_byte;
        while (NRF_UART0->EVENTS_TXDRDY != 1);
        NRF_UART0->EVENTS_TXDRDY = 0;
        nrf_gpio_pin_write(PIN_DEF_BLE_LED_EN, out_lvl);
        out_lvl = !out_lvl;
    };
}
#endif

/**@brief Function for application main entry.
 *
 * @details This function serves as an adaptation layer between a 2-wire UART interface and the
 *          dtmlib. After initialization, DTM commands submitted through the UART are forwarded to
 *          dtmlib and events (i.e. results from the command) is reported back through the UART.
 */

int dtm_mode(void)
{
    uint32_t    current_time;
    uint32_t    dtm_error_code;
    uint32_t    msb_time          = 0;     // Time when MSB of the DTM command was read. Used to catch stray bytes from "misbehaving" testers.
    bool        is_msb_read       = false; // True when MSB of the DTM command has been read and the application is waiting for LSB.
    uint16_t    dtm_cmd_from_uart = 0;     // Packed command containing command_code:freqency:length:payload in 2:6:6:2 bits.
    uint8_t     rx_byte;                   // Last byte read from UART.
    dtm_event_t result;                    // Result of a DTM operation.

    nrf_gpio_cfg_output(PIN_DEF_BLE_LED_EN);
    nrf_gpio_pin_set(PIN_DEF_BLE_LED_EN);

    nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
    nrf_gpio_pin_clear(PIN_DEF_2ND_POW_EN);

#if 0  //goto bootloaer for fota
    bootloader_check_enter();
#endif

    uart_init();

#ifdef TEST_FEATURE_UART_ECHO
    uart_echo_test();
#endif

    dtm_error_code = dtm_init();
    if (dtm_error_code != DTM_SUCCESS)
    {
        // If DTM cannot be correctly initialized, then we just return.
        return -1;
    }

    for (;;)
    {
        // Will return every timeout, 625 us.
        current_time = dtm_wait();

        if (NRF_UART0->EVENTS_RXDRDY == 0)
        {
            // Nothing read from the UART.
            continue;
        }
        NRF_UART0->EVENTS_RXDRDY = 0;
        rx_byte                  = (uint8_t)NRF_UART0->RXD;

#ifdef FEATURE_TEST_LED_ON_OFF
        if(rx_byte == 'a')
        {
            nrf_gpio_pin_set(PIN_DEF_BLE_LED_EN);
        }
        else if(rx_byte == 'b')
        {
            nrf_gpio_pin_clear(PIN_DEF_BLE_LED_EN);
        }
#endif
        if (!is_msb_read)
        {
            // This is first byte of two-byte command.
            is_msb_read       = true;
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;

            // Go back and wait for 2nd byte of command word.
            continue;
        }

        // This is the second byte read; combine it with the first and process command
        if (current_time > (msb_time + MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE))
        {
            // More than ~5mS after msb: Drop old byte, take the new byte as MSB.
            // The variable is_msb_read will remains true.
            // Go back and wait for 2nd byte of the command word.
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;
            continue;
        }

        // 2-byte UART command received.
        is_msb_read        = false;
        dtm_cmd_from_uart |= (dtm_cmd_t)rx_byte;

        if (dtm_cmd_put(dtm_cmd_from_uart) != DTM_SUCCESS)
        {
            // Extended error handling may be put here.
            // Default behavior is to return the event on the UART (see below);
            // the event report will reflect any lack of success.
        }

        // Retrieve result of the operation. This implementation will busy-loop
        // for the duration of the byte transmissions on the UART.
        if (dtm_event_get(&result))
        {
            // Report command status on the UART.
            // Transmit MSB of the result.
            NRF_UART0->TXD = (result >> 8) & 0xFF;
            // Wait until MSB is sent.
            while (NRF_UART0->EVENTS_TXDRDY != 1)
            {
                // Do nothing.
            }
            NRF_UART0->EVENTS_TXDRDY = 0;

            // Transmit LSB of the result.
            NRF_UART0->TXD = result & 0xFF;
            // Wait until LSB is sent.
            while (NRF_UART0->EVENTS_TXDRDY != 1)
            {
                // Do nothing.
            }
            NRF_UART0->EVENTS_TXDRDY = 0;
        }
    }
}

/// @}
