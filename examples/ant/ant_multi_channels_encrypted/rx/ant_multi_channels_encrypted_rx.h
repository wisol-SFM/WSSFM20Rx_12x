/*
This software is subject to the license described in the license.txt file included with
this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef ANT_MULTI_CHANNELS_ENCRYPTED_RX_H__
#define ANT_MULTI_CHANNELS_ENCRYPTED_RX_H__

#include <stdint.h>
#include "softdevice_handler.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Function to show the number of currently tracking channels as a binary number on the LEDs.
 */
void ant_se_num_of_decrypted_channels_display(void);

/**@brief Function for setting up the ANT module to be ready for RX broadcast.
 *
 * The following commands are issued in this order:
 * - Enable the number of channels specified in NUMBER_OF_CHANNELS_TO_OPEN
 * - Set up encryption
 * - Loop through the channels and perform the following commands on each channel:
 *      - Assign channel
 *      - Set channel period
 *      - Set RF frequency
 *      - Set channel ID
 *      - Open channel
 */
void ant_se_channel_rx_broadcast_setup(void);

/**@brief Function for handling application specific events.
 *
 * @param[in] p_ant_evt  Pointer to ANT stack event message structure.
 */
void ant_se_event_handler(ant_evt_t * p_ant_event);



#ifdef __cplusplus
}
#endif

#endif // ANT_MULTI_CHANNELS_ENCRYPTED_RX_H__
