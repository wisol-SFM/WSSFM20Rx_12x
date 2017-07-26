/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef ANT_SCALEABLE_RX_H__
#define ANT_SCALEABLE_RX_H__

#include <stdint.h>
#include "ant_stack_handler_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Function to show the number of currently tracking channels as a binary number on the LEDs.
 *
 */
void ant_scaleable_display_num_tracking_channels(void);

/**@brief Function for setting up and opening channels to be ready for RX broadcast.
 *
 */
void ant_scaleable_channel_rx_broadcast_setup(void);

/**@brief Handle application specific events
 * @param[in] p_ant_evt A pointer to the received ANT event to handle.
 */
void ant_scaleable_event_handler(ant_evt_t * p_ant_evt);


#ifdef __cplusplus
}
#endif

#endif // ANT_SCALEABLE_RX_H__
