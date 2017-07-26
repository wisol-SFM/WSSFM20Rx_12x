/*
This software is subject to the license described in the license.txt file included with
this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef ANT_MULTI_CHANNELS_ENCRYPTED_TX_H__
#define ANT_MULTI_CHANNELS_ENCRYPTED_TX_H__

#include <stdint.h>
#include "softdevice_handler.h"

#ifdef __cplusplus
extern "C" {
#endif


/**@brief Function for handling an ANT stack event.
 *
 * This function is called from the ANT stack event interrupt handler after an ANT stack
 * event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_multi_channels_encrypted_event_handler(ant_evt_t * p_ant_evt);

/**@brief Function for setting up the ANT module to be ready for TX broadcast.
 *
 * The following commands are issued in this order:
 * - Enable the number of channels specified in NUMBER_OF_CHANNELS_TO_OPEN
 * - Set up encryption
 * - Loop through the channels and perform the following commands on each channel:
 *      - Assign channel
 *      - Enable encryption on this channel
 *      - Set channel period
 *      - Set RF frequency
 *      - Set channel ID
 *      - Open channel
 */
void ant_multi_channels_encrypted_channel_tx_broadcast_setup(void);


#ifdef __cplusplus
}
#endif

#endif // ANT_MULTI_CHANNELS_ENCRYPTED_TX_H__
