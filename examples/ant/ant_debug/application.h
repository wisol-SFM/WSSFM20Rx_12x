/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/


#ifndef APPLICATION_H__
#define APPLICATION_H__

/**
 * @file application.h
 * @brief Example application header file
 * @ingroup application
 */

#include <stdint.h>
#include "ant_stack_handler_types.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APPLICATION_CHANNEL                       0x00        /**< ANT Channel 0. */

/**@brief Function for configuring and opening master channel.
 *
 */
void app_channel_setup(void);

/**@brief Function for processing channel event messages
 *
 * @param[in] p_ant_evt A pointer to the received ANT event to handle.
 */
void app_channel_event_handler(ant_evt_t * p_ant_evt);

#if DEBUG_CHANNEL_INCLUDED
void app_custom_debug_command_handler(uint8_t const * const p_command);
#endif // DEBUG_CHANNEL_INCLUDED


#ifdef __cplusplus
}
#endif

#endif // APPLICATION_H__
