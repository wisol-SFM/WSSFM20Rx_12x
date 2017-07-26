/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef ANT_DEBUG_H__
#define ANT_DEBUG_H__

/**
 * @file debug.h
 * @brief Debug module header
 * @ingroup debug
 */

#include <stdint.h>
#include <stdbool.h>
#include "ant_stack_handler_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_CHANNEL                           ((uint8_t) 1)       /**< Debug channel number. */

// Debug field definitions
#define ANT_DEBUG_FIELD_BUTTON_A_STATUS         ((uint8_t) 0)       /**< Status button A. */
#define ANT_DEBUG_FIELD_BUTTON_B_STATUS         ((uint8_t) 1)       /**< Status button B. */
#define ANT_DEBUG_FIELD_BUTTON_C_STATUS         ((uint8_t) 2)       /**< Status button C. */
#define ANT_DEBUG_FIELD_BUTTON_D_STATUS         ((uint8_t) 3)       /**< Status button D. */
#define ANT_DEBUG_FIELD_ERR_LINE                ((uint8_t) 4)       /**< Error line. */
#define ANT_DEBUG_FIELD_FILE_NAME               ((uint8_t) 5)       /**< File name. */
#define ANT_DEBUG_FIELD_GPIO_REGISTER_LOW       ((uint8_t) 6)       /**< GPIO register low. */
#define ANT_DEBUG_FIELD_GPIO_REGISTER_HIGH      ((uint8_t) 7)       /**< GPIO register high. */

#define ANT_DEBUG_FIELD_RX_TOTAL_CH0            ((uint8_t) 132)     /**< Total Rx channel 0. */
#define ANT_DEBUG_FIELD_TX_TOTAL_CH0            ((uint8_t) 140)     /**< Total Tx channel 0. */
#define ANT_DEBUG_FIELD_COLLISIONS_CH0          ((uint8_t) 148)     /**< Total collisions channel 0.*/


/**@brief Function prototype for handler of reverse direction commands on debug channel*/
typedef void (*custom_command_handler_t)(uint8_t const * const p_command);


/** @brief Function to initialize the debug channel.
 *
 * Sets up debug queue and opens channel.
 * Device number will be set the lower part of the serial number
 */
void ad_init(void);

/**
 * @brief Register a callback function for custom commands sent over the debug channel.
 * This allows custom commands received over the debug channel to be used by the main application.
 *
 * @param[in] callback  The pointer to a function which will be called when a custom command is received
 */
void ad_custom_command_callback_register(custom_command_handler_t callback);

/**
 * @brief Function to set the value of a specific debug field
 *
 * @param[in] index     Index of debug field to be set
 * @param[in] value     New field value
 */
void ad_debug_field_set(uint8_t index, uint16_t field_value);

/**
 * @brief Retrieve the value of a particular debug field
 *
 * @param[in]  index            Index of value to retrieve
 * @param[out] p_field_value    Retrieved value
 */
bool ad_debug_field_get(uint8_t index, uint16_t * p_field_value);

/**
 * @brief Adds 1 to the value of a debug field
 *
 * @param[in] index           Index of the field to be incremented
 */
void ad_debug_field_increment(uint8_t index);

/**
 * @brief Sets the fast debug byte
 *
 * @param[in] fdb_value       Value to assign to the fast debug byte
 */
void ad_fast_debug_byte_set(uint8_t fdb_value);

/**
 * @brief Handles ANT events coming in on the DEBUG_CHANNEL
 *
 * Handles any filter commands sent from the debug tool.
 * Sends the debug messages on the debug channel.
 *
 * @param[in] p_ant_event     Event to be handled
 */
void ad_ant_event_process(ant_evt_t * p_ant_evt);

/**
 * @brief Forces debug channel to transmit to this error code, file name and line number.
 * Use for assert functions only.
 *
 * @param[in] error_code       Error code
 * @param[in] error_line       Line number where error occurred
 * @param[in] p_file_name      Pointer to char array containing file name where error occured
 */
void ad_error_page_force(uint8_t error_code, uint16_t error_line, const char * p_file_name);


#ifdef __cplusplus
}
#endif

#endif // ANT_DEBUG_H__

